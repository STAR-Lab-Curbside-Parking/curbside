import curbside
import gym
import numpy as np
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

class RingEnv(gym.Env):
    
    def __init__(self, net_xml, add_xml, rou_xml, window=10, gui=False):
        """
        initialization
        """
        
        # general info
        self.NET_XML = net_xml
        self.ADD_XML = add_xml
        self.ROU_XML = rou_xml
        self.WINDOW = window
        self.GUI = gui
        
        self.TIME = 0

        self.curbs = self._init_curbs()
        self.sim = self._init_sim(self.GUI)

        self.v_edge = set()
        
        self.demand = {key:{'cv':0, 'ncv':0} for key in self.curbs}


    def _init_curbs(self):
        """
        initialize the curbs and how they find their neighbors on the defined network
        """

        curb_ids = ['P01', 'P12', 'P23', 'P30']

        curbs = {}
        for curb_id in curb_ids:
            curbs[curb_id] = curbside.smart_curb(curb_id, 10)

        return curbs
    
    def _init_sim(self, gui):
        """
        initialize simulation

        Args:
            gui : boolean, whether to run the simulation with gui
        """
        if sys.platform == "win32":
            if self.GUI:
                sumoBinary = checkBinary('sumo-gui')
            else:
                sumoBinary = checkBinary('sumo')
        else:
            if self.GUI:
                sumoBinary = "sumo-gui"
            else:
                sumoBinary = "sumo"

        traci.start([sumoBinary, "-c", "ring.sumocfg", "--time-to-teleport", "-1"], label="sim")
        return traci.getConnection("sim")
    
    def _simulate(self):
            
        for _ in range(self.WINDOW):
            
            self.sim.simulationStep()

            self.TIME += 1

            self._micro()
            
    def _micro(self):
        """
        control the parking game via curbs, but need to tell each vehicle what to do
        """
        
        for curb in self.curbs.values():
            curb._update_neigh_occupy(self.curbs)
            
            # check if there is any vehicle enters
            v_enter = set(self.sim.edge.getLastStepVehicleIDs(curb.edge)).difference(self.v_edge)
            self.v_edge = set(self.sim.edge.getLastStepVehicleIDs(curb.edge))
            
            print(v_enter)

            for veh in v_enter:
                
                # vehicle entering can be of 3 types (b, ncv, cv)
                # first filter on those who actually are parking at this lane
                # for those passers-by, do nothing
                if not veh.startswith('b', 0, 1)\
                   and self.sim.vehicle.getNextStops(veh) \
                   and curb.id in [item[2] for item in self.sim.vehicle.getNextStops(veh)]:

                    # only consider those early in the lane
                    if self.sim.vehicle.getLanePosition(veh) < curb.start_pos:

                        # vehicle from last t is duplicated
                        print("Hello")
                        print(curb.parked_veh)
                        assert veh not in curb.parked_veh
                        
                        # vclass: passenger or delivery
                        vclass = self.sim.vehicle.getVehicleClass(veh)
                        if vclass == 'delivery':
                            occ = curb.cv_occ
                            cap = curb.cv_cap
                        else:
                            occ = curb.ncv_occ
                            cap = curb.ncv_cap
                            
                        if occ < cap:
                            
                            # if can park, add to occupied set : planned + parked
                            curb.parked_veh.add(veh)
                            # update curb occ and cap immediately when a vehicle is accepted
                            # multiple vehicles can enter in the same second
                            curb._occupy_cnt()

                        else: # not enough space at this curb

                            reroute_curb = curb._reroute_choice(veh, self.curbs)
                            
                            # change destination to next exit
                            self.sim.vehicle.changeTarget(veh, "E" + reroute_curb[1:])

                            # reroute to neighbor curb
                            self.sim.vehicle.rerouteParkingArea(veh, reroute_curb)
                            
                            
            # update parked vehicle set
            # assert curb.parked_veh == set(self.sim.parkingarea.getVehicleIDs(curb.id))
            curb._occupy_cnt()
            
    def _get_state(self):
        """
        retrieve state of each curb
        includes its state tuple (cv_occ, ncv_occ, cv_cap, ncv_cap) and all of its neighbors'
        ensure the output state size is 12

        needs design 07.22
        """
        
        state = []
        
        for i in range(len(self.curbs)):
            curb = self.curbs[i]

            tmp = np.array([self.TIME, 
                            # average driving distance by vehicles
                            # critic sees CV + non-CV: cv and ncv
                            curb.cv_occ, curb.ncv_occ, curb.cv_cap, curb.ncv_cap])
            
            state.append(tmp)

        return state
    
    def _get_reward(self):
        """
        retrieve reward for the curbs
        """
        reward = []
        # reroute is collected for reward calculation
        for i in range(len(self.curbs)):
            curb = self.curbs[i]

            # reward is calculated of both CV + non-CV: cv and ncv
            reward.append()

        return reward
    
    def control(self, actions):
        """
        perform control of the curbs
        """

        # use this to ensure sequence is matched
        for i in range(len(self.curbs)):
            curb = self.curbs[i]
            action = actions[i]
            if action == 1 and curb.ncv_occ < curb.ncv_cap:
                # delivery vehicle space +1
                curb.ncv_cap -= 1
                curb.cv_cap += 1
            elif action == -1 and curb.cv_occ < curb.cv_cap:
                # passenger vehicle space +1
                curb.ncv_cap += 1
                curb.cv_cap -= 1
    
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
        
        self.control(actions)

        # _simulate() wraps for (control_window) steps behind the scene
        self._simulate()

        # state is the most recent
        # reward is calculated based on reroute per batch

        state = self._get_state()
        reward = self._get_reward()

        return state, reward

    
    def terminate(self):
        traci.close()
        self.close()


