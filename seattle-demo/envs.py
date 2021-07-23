import curbside
import utils
import gym
import numpy as np
import os, sys
import scipy.optimize
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
    
    def __init__(self, net_xml, add_xml, rou_xml, dlv_prc, psg_prc, curb_control=True, gui=False):
        """
        initialization
        """
        
        # general info
        self.net_xml = net_xml
        self.add_xml = add_xml
        self.rou_xml = rou_xml
        self.gui = gui
        
        self.time_step = 0

        # matching or curb control
        self.curb_control = curb_control
        
        # if matching mode, then initiate a different set of features
        if not self.curb_control:
            self.assigned_veh = set()
            self.control_window = 3
            self.curbs, self.curb_ids = self._init_curbs(option='whole')
            self.skip_dest = {}
        else:
            self.control_window = 60
            self.curbs, self.curb_ids = self._init_curbs(option='nearest')
        
        # reroute and failed metrics
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
        
        self.sim = self._init_sim(self.gui)

    
    def _init_curbs(self, option='nearest'):
        """
        initialize the curbs and how they find their neighbors on the defined network
        """

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
            curb.find_neighborhood(road_network, curbs, option)
            # curb.dlv_cap = curb.tot_cap
        return curbs, curb_ids
    
    def _init_sim(self, gui):
        """
        initialize simulation

        Args:
            gui : boolean, whether to run the simulation with gui
        """
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

        traci.start([sumoBinary, "-c", "seattle.sumocfg", "--time-to-teleport", "-1"], label="sim")
        return traci.getConnection("sim")
    
    def _simulate(self):
            
        for _ in range(self.control_window):
            
            self.sim.simulationStep()

            self.time_step += 1

            self._micro()
        
            
    def _micro(self):
        """
        control the parking game via curbs
        """
        
        # check every time step to end cruising
        if not self.curb_control:
            finish_reassign = []
            for veh, skip_info in self.skip_dest.items():
                if self.sim.vehicle.getRoadID(veh) != skip_info[0]:
                    
                    # if passed the parking edge, re-add the destination
                    # self.sim.vehicle.highlight(veh, color=(0, 255, 0), size=10)
                    self.sim.vehicle.setParkingAreaStop(veh, stopID=skip_info[1], duration=skip_info[2])
                    
                    # append it to the reassigned list (i.e., cruise cancelling list)
                    finish_reassign.append(veh)
            
            # remove those reassigned
            for item in finish_reassign:
                del self.skip_dest[item]
        
        
        for curb in self.curbs.values():
            curb._update_neigh_occupy(self.curbs)
            
            # check if there is any vehicle enters
            v_enter = set(self.sim.edge.getLastStepVehicleIDs(curb.edge)) - curb.moving_vehicle
            
            for veh in v_enter:
                
                # vehicle entering can be of 4 types (f, exist, psg, dlv)
                if not veh.startswith('f', 0, 1)\
                   and self.sim.vehicle.getNextStops(veh) \
                   and curb.id in [item[2] for item in self.sim.vehicle.getNextStops(veh)]:

                    # if parking area is in future stop list
                    # check stop duration > 1
                    if self.sim.vehicle.getStopState(veh) != 131:
                        
                        if not self.curb_control and not veh.startswith('exist', 0, 5) and veh not in self.assigned_veh:
                            
                            # record this in a separate dictionary and add the planned the destination back at next edge
                            _, _, planned_park_dest, _, park_leng, _ = self.sim.vehicle.getNextStops(veh)[0]
                            self.skip_dest[veh] = (curb.edge, planned_park_dest, park_leng)

                            # cancel the current stop first, then add it back when the vehicle reaches a different edge
                            # this is for cruise behavior
                            self.sim.vehicle.setParkingAreaStop(veh, stopID=curb.id, duration=0)
                            
                            self.sim.vehicle.highlight(veh, color=(0, 255, 0), size=20)
                            
                            print(veh)

                        else:
                            # if trip is not ending and the current edge is the parking stop, v_leave should be excluded
                            
                            # vclass: passenger or delivery
                            vclass = self.sim.vehicle.getVehicleClass(veh)
                            if vclass == 'delivery':
                                occ = curb.dlv_occ
                                cap = curb.dlv_cap
                            else:
                                occ = curb.psg_occ
                                cap = curb.psg_cap
                                
                            # deals with both matching and curb mode
                            if occ < cap or veh in curb.occupied_vehicle:
                                
                                # in matching mode, vehicle is already recorded in occupied vehicle information
                                # so occ<cap is often violated
                                # if reserved early, it should be allowed by (veh in curb.occupied_vehicle)
                                
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
                                    # self.sim.vehicle.highlight(veh, size=20)
                                    # ask it to leave
                                    self.sim.vehicle.setParkingAreaStop(veh, stopID=curb.id, duration=0)

                                    self.failed_total += 1 # number of failed parking + 1
                                    if vclass == 'delivery':
                                        self.failed_vtype[curb.id]['dlv'] += 1
                                    else:
                                        self.failed_vtype[curb.id]['psg'] += 1
                                else:
                                    # if does not reach reroute limit
                                    # print('here veh:{}'.format(veh))
                                    # print(self.sim.vehicle.getNextStops(veh))

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

        needs design 07.22
        """
        
        state = []
        
        for i in range(len(self.curb_ids)):
            curb = self.curbs[self.curb_ids[i]]

            # 07.22, state of shape 7*1, selfish
            tmp = np.array([self.time_step, \
                            self.reroute_vtype_step[self.curb_ids[i]]['dlv'], self.reroute_vtype_step[self.curb_ids[i]]['psg'], \
                            curb.dlv_occ, curb.psg_occ, curb.dlv_cap, curb.psg_cap])

            # 02.01, state of shape 12*4, including neighbors' states

            # # first row: dlv reroute
            # tmp = np.ones(4) * self.reroute_vtype_step[self.curb_ids[i]]['dlv']
            # # second row: psg reroute
            # tmp = np.vstack((tmp, np.ones(4) * self.reroute_vtype_step[self.curb_ids[i]]['psg']))
            # # third row self occupancy and capacity
            # tmp = np.vstack((tmp, np.array([curb.dlv_occ, curb.psg_occ, curb.dlv_cap, curb.psg_cap])))
            # # onward: neighbors occupancy and capacity
            # for _, neighbor_condition in curb.neighbor.items():
            #     tmp = np.vstack((tmp, np.array(neighbor_condition)))
            
            # while tmp.shape[0] < 12:
            #     tmp = np.vstack((tmp, np.zeros(4)))
            
            state.append(tmp)

        return state
    
    def _get_reward(self):
        """
        retrieve reward for the curbs, defined as price of reroute

        needs re-design 07.22
        """
        reward = {}

        # reroute is collected for reward calculation
        for curb_id in self.reroute_vtype_step:
            reward[curb_id] = self.dlv_prc * self.reroute_vtype_step[curb_id]['dlv'] + self.psg_prc * self.reroute_vtype_step[curb_id]['psg']
        
        return reward
    
    def control(self, actions):
        """
        perform control of the curbs
        """

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
    
    def batch(self, actions):
        """
        perform control and collect information every batch of seconds

        Args:
            actions : dictionary, actions associated with each curb agent, computed outside of the env

        Returns:
            state : dictionary, state of each curb agent
            reward : dictionary, reward of each curb agent
            global_reward : global_reward
        """
        
        if self.curb_control:
            # curb control mode

            self.control(actions)
        
        else:

            # two matching games
            dlv_curb_cand = []
            psg_curb_cand = []
            
            # add available curbs to one side first
            for curb in self.curbs.values():
                # lazy occupy count
                curb._occupy_cnt()
                if curb.dlv_occ < curb.dlv_cap:
                    dlv_curb_cand.append(curb)
                if curb.psg_occ < curb.psg_cap:
                    psg_curb_cand.append(curb)
            
            # then add unmatched vehicles to the other side
            veh_list_step = set(self.sim.vehicle.getIDList())
            dlv_list_step = [item for item in veh_list_step if item.startswith('dlv', 0, 3) \
                             and item not in self.assigned_veh \
                             and self.sim.vehicle.getNextStops(item) \
                             and self.sim.vehicle.getStopState(item) != 131]
            psg_list_step = [item for item in veh_list_step if item.startswith('psg', 0, 3) \
                             and item not in self.assigned_veh \
                             and self.sim.vehicle.getNextStops(item) \
                             and self.sim.vehicle.getStopState(item) != 131]
            
            if len(dlv_list_step) > 0:
                dlv_mch_veh, dlv_mch_curb = self._match(dlv_list_step, dlv_curb_cand, 'dlv')
                self.assigned_veh = self.assigned_veh.union(set(dlv_mch_veh))
            
            if len(psg_list_step) > 0:
                psg_mch_veh, psg_mch_curb = self._match(psg_list_step, psg_curb_cand, 'psg')
                self.assigned_veh = self.assigned_veh.union(set(psg_mch_veh))
        
        self.reroute_vtype_step = {curb_id:{'dlv':0, 'psg':0} for curb_id in self.curb_ids}

        # _simulate() wraps for (control_window) steps behind the scene
        self._simulate()

        # state is the most recent
        # reward is calculated based on reroute per batch

        state = self._get_state()
        reward = self._get_reward()

        return state, reward

    def _match(self, veh_list, curb_list, flag):
        """
        matching function, not used if self.curb_control is TRUE
        """
        
        matching = np.zeros((len(veh_list), len(curb_list)))
        
        for i in range(len(veh_list)):
            _, _, planned_park_dest, _, park_leng, _ = self.sim.vehicle.getNextStops(veh_list[i])[0]
            assert park_leng >= 180
            
            for j in range(len(curb_list)):
                # distance from planned curb to available curbs
                if planned_park_dest != curb_list[j].id:
                    matching[i,j] = [item[1] for item in self.curbs[planned_park_dest].neighbor.keys() if item[0] == curb_list[j].id][0]
                else:
                    matching[i,j] = 0
        
        row_ind, col_ind = scipy.optimize.linear_sum_assignment(matching)
        
        # reroute and to curb record directly
        for i in range(len(row_ind)):
            self.sim.vehicle.rerouteParkingArea(veh_list[row_ind[i]], curb_list[col_ind[i]].id)
            cur_curb = curb_list[col_ind[i]]
            cur_curb.occupied_vehicle.add(veh_list[row_ind[i]])
            # update curb occ and cap immediately when a vehicle is accepted
            # multiple vehicles can enter in the same second
            cur_curb._occupy_cnt()
        
        return [veh_list[i] for i in row_ind], [curb_list[i] for i in col_ind]
    
    def terminate(self):
        traci.close()


