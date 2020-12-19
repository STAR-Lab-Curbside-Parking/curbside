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
    
    def __init__(self, net_xml, add_xml, rou_xml, dlv_prc, psg_prc, gui=False):
        self.net_xml = net_xml
        self.add_xml = add_xml
        self.rou_xml = rou_xml
        self.curbs, self.curb_ids = self._init_curbs()
        self.gui = gui
        self.curr_episode = 0
        self.sim = self._init_sim(self.gui)
        
        self.time_step = 0
        self.control_window = 60
        
        self.reroute_total = 0
        self.reroute_curb = dict(zip(self.curb_ids, np.zeros(len(self.curb_ids), dtype=int)))
        self.reroute_vtype = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}
        self.reroute_veh = {}
        self.reroute_limit = 3
        
        self.failed_total = 0
        self.failed_vtype = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}
        
        self.dlv_prc = dlv_prc
        self.psg_prc = psg_prc
        
        self.reroute_vtype_step = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}
        
    
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
            
        for _ in range(self.control_window):
            self.sim.simulationStep()
            
            # if 'psg_310' in self.sim.simulation.getDepartedIDList():
            #     self.sim.vehicle.highlight('psg_310', size=20)

            self.time_step += 1
            for curb in self.curbs.values():
                curb._update_neigh_occupy(self.curbs)
                # check if there is any vehicle enters
                v_enter = set(self.sim.edge.getLastStepVehicleIDs(curb.edge)) - curb.moving_vehicle
                for veh in v_enter:
                    if not veh.startswith('f', 0, 1)\
                       and self.sim.vehicle.getNextStops(veh) \
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
                                # update curb occ and cap immediately when a vehicle is accepted
                                # multiple vehicles can enter in the same second
                                curb._occupy_cnt()
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
                                    self.reroute_vtype_step[curb.id]['dlv'] += 1
                                else:
                                    self.reroute_vtype[curb.id]['psg'] += 1
                                    self.reroute_vtype_step[curb.id]['psg'] += 1

                                # actually reroute
                                if self.reroute_veh[veh] >= self.reroute_limit:
                                    self.sim.vehicle.highlight(veh, size=20)
                                    # ask it to leave
                                    self.sim.vehicle.setParkingAreaStop(veh, stopID=curb.id, duration=0)
                                    
                                    self.failed_total += 1 # number of failed parking + 1
                                    if vclass == 'delivery':
                                        self.failed_vtype[curb.id]['dlv'] += 1
                                    else:
                                        self.failed_vtype[curb.id]['psg'] += 1
                                else:
                                    # if does not reach reroute limit
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

    def _get_state(self):
        """
        retrieve state of each curb
        includes its state tuple (dlv_occ, psg_occ, dlv_cap, psg_cap) and all of its neighbors'
        ensure the output state size is 12
        """
        
        state = []
        
        for i in range(len(self.curb_ids)):
            curb = self.curbs[self.curb_ids[i]]
            # first row: dlv reroute
            tmp = np.ones(4) * self.reroute_vtype_step[self.curb_ids[i]]['dlv']
            # second row: psg reroute
            tmp = np.vstack((tmp, np.ones(4) * self.reroute_vtype_step[self.curb_ids[i]]['psg']))
            # third row self occupancy and capacity
            tmp = np.vstack((tmp, np.array([curb.dlv_occ, curb.psg_occ, curb.dlv_cap, curb.psg_cap])))
            # onward: neighbors occupancy and capacity
            for _, neighbor_condition in curb.neighbor.items():
                tmp = np.vstack((tmp, np.array(neighbor_condition)))
            
            while tmp.shape[0] < 12:
                tmp = np.vstack((tmp, np.zeros(4)))
            
            state.append(tmp)

        return state
    
    def _get_reward(self):
        
        reward = {}
        for curb_id in self.reroute_vtype_step:
            reward[curb_id] = self.dlv_prc * self.reroute_vtype_step[curb_id]['dlv'] + self.psg_prc * self.reroute_vtype_step[curb_id]['psg']
        
        return reward
    
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
    
    def batch(self, actions, seconds):
        self.control(actions)
        
        self.reroute_vtype_step = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}
        
        self._simulate()
        
        state = self._get_state()
        reward = self._get_reward()
        global_reward = sum(reward.values())

        done = False
        # if accumulated batch reaches the simulation time, then simulation should be over
        if self.time_step >= seconds:
            done = True
        return done, state, reward, global_reward

    def reset(self):
        self.time_step = 0
        self.curr_episode += 1
        
        return self._get_state()
    
    def terminate(self):
        traci.close()


