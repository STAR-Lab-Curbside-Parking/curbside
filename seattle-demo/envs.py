import curbside
import utils
import numpy as np, gym
import os, sys
import xml.etree.ElementTree as ET

if sys.platform == "win32":
    # windows, win32
    from sumolib import checkBinary
else:
    # mac, darwin
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    
import traci

class SeattleEnv(gym.Env):
    
    def __init__(self, net_xml, add_xml, rou_xml, gui=False):
        self.net_xml = net_xml
        self.add_xml = add_xml
        self.rou_xml = rou_xml
        self.curbs, self.curb_ids = self._init_curbs()
        self.gui = gui
        self.sim = self._init_sim(self.gui)
        
        self.time_step = 0
        self.control_window = 60
        
        self.reroute_total = 0
        self.reroute_curb = dict(zip(self.curb_ids, np.zeros(len(self.curb_ids), dtype=int)))
        self.reroute_vtype = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}
        self.reroute_veh = {}
        self.reroute_limit = 3
        
    
    def _init_curbs(self):
        road_network = utils.create_graph(self.net_xml)
        # create agent curbs
        curb_ids = []
        root = ET.parse(self.add_xml).getroot()
        for child in root.iter('additional'):
            for kid in child.iter('parkingArea'):
                curb_ids.append(kid.get('id'))
        curbs = {}
        for curb_id in curb_ids:
            curbs[curb_id] = curbside.SmartCurbside(1, self.add_xml, self.net_xml, curb_id, 
                                                    ['passenger', 'delivery'], road_network)
        for curb in curbs.values():
            curb.find_neighborhood(road_network, curbs)
            # curb.dlv_cap = curb.tot_cap
        return curbs, curb_ids
    
    def _init_sim(self, gui):
        if sys.platform == "win32":
            if self.gui:
                sumoBinary = checkBinary('sumo-gui')
            else:
                sumoBinary = checkBinary('sumo')
        else:
            if self.gui:
                sumoBinary = "sumo-gui"
            else:
                sumoBinary = "sumo"

        traci.start([sumoBinary, "-c", "seattle.sumocfg", "--time-to-teleport", "-1"], label="sim1")
        return traci.getConnection("sim1")
    
    def _simulate(self):
        if self.time_step >= 13:
            self.sim.vehicle.highlight('dlv_0', size=20)
            
        for _ in range(self.control_window):
            self.sim.simulationStep()
            self.time_step += 1
            for curb in self.curbs.values():
                curb._update_neigh_occupy(self.curbs)
                # check if there is any vehicle enters
                v_enter = set(self.sim.edge.getLastStepVehicleIDs(curb.edge)) - curb.moving_vehicle
                for veh in v_enter:
                    if self.sim.vehicle.getNextStops(veh) \
                        and curb.id in [item[2] for item in self.sim.vehicle.getNextStops(veh)]:

                        # if parking area is in future stop list
                        # check stop duration > 1
                        stop_idx = [item[2] for item in self.sim.vehicle.getNextStops(veh)].index(curb.id)
                        if [item[4] for item in self.sim.vehicle.getNextStops(veh)][stop_idx] > 1:

                            # if trip is not ending and the current edge is the parking stop, v_leave should be excluded
                            # vclass: passenger or delivery
                            vclass = self.sim.vehicle.getVehicleClass(veh)
                            if vclass == 'delivery':
                                occ = curb.dlv_occ
                                cap = curb.dlv_cap
                            else:
                                occ = curb.psg_occ
                                cap = curb.psg_cap
                            if occ < cap:
                                # if can park, add to occupied set : planned + parked
                                curb.occupied_vehicle.add(veh)
                            else:
                                # cannot park at this edge, reroute
                                # item._reroute_choice() returns (curb_id, distance) tuple
                                reroute_msg = curb._reroute_choice(veh, self.curbs)

                                # track reroute num per veh
                                if veh in self.reroute_veh:
                                    self.reroute_veh[veh] += 1
                                else:
                                    self.reroute_veh[veh] = 1
                                
                                self.reroute_total += 1
                                self.reroute_curb[curb.id] += 1
                                if vclass == 'delivery':
                                    self.reroute_vtype[curb.id]['dlv'] += 1
                                else:
                                    self.reroute_vtype[curb.id]['psg'] += 1

                                # actually reroute
                                if self.reroute_veh[veh] > self.reroute_limit:
                                    # leave
                                    self.sim.vehicle.rerouteTraveltime(veh, currentTravelTimes=True)
                                    # self.sim.vehicle.highlight(veh, size=20)
                                    print(veh)
                                    print(self.sim.vehicle.getNextStops(veh))
                                else:
                                    self.sim.vehicle.rerouteParkingArea(veh, reroute_msg[0])

                                # reroute_cost[curb_id] += reroute_msg[1]
                            
                # v_leave : parked vehicle of last time step - parked vehicle at this time step
                v_leave = curb.parked_vehicle - set(self.sim.parkingarea.getVehicleIDs(curb.id))
                # remove v_leave from occupied set : planned + parked
                curb.occupied_vehicle -= v_leave
                # update parked vehicle set
                curb.parked_vehicle = set(self.sim.parkingarea.getVehicleIDs(curb.id))
                # update moving vehicles on the hosting edge
                curb.moving_vehicle = set(self.sim.edge.getLastStepVehicleIDs(curb.edge))
                # update two-type occupancy for next time step
                curb._occupy_cnt()

    def control(self, actions):
        # use this to ensure sequence is matched
        for i in range(len(self.curb_ids)):
            curb = self.curbs[self.curb_ids[i]]
            action = actions[i]
            if action == 1 and curb.psg_cap > 0 and curb.psg_occ < curb.psg_cap:
                # delivery vehicle space +1
                curb.psg_cap -= 1
                curb.dlv_cap += 1
            elif action == -1 and curb.dlv_cap > 0 and curb.dlv_occ < curb.dlv_cap:
                # passenger vehicle space +1
                curb.psg_cap += 1
                curb.dlv_cap -= 1
    
    def step(self, actions, seconds):
        self.control(actions)
        self._simulate()
        print('reroute total:', self.reroute_total)

        done = False
        if self.time_step >= seconds:
            done = True
        return done

    def terminate(self):
        self.sim.close()


class Policy:
    def __init__(self, curb_ids):
        self.curb_ids = curb_ids
        
    def forward(self, reroute_vtype):
        actions = []
        for curb_id in self.curb_ids:
            reroute_dict = reroute_vtype[curb_id]
            if reroute_dict['dlv'] > reroute_dict['psg']:
                action = 1 # delivery vehicle space +1
            elif reroute_dict['dlv'] < reroute_dict['psg']:
                action = -1 # passenger vehicle space +1
            else:
                action = 0
            actions.append(action)
        return actions
