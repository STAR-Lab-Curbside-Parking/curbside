import gym, numpy as np, random
import os, sys

import curbside

random.seed(0)

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
        initialization of object

        @params net_xml : str, path to SUMO network file
        @params add_xml : str, path to SUMO additional facility file, \
            where parking areas are induction loops are defined for simulation
        @params rou_xml : str, path to SUMO route file, vehicle routes defined there
        @params window : int, control window length, e.g., control every 10 seconds
        @params gui : bollean, if simulation runs with gui or not
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
        
        self.demand = {key:{'cv':0, 'ncv':0} for key in self.curbs}


    def _init_curbs(self):
        """
        initialize the curb objects in simulation
        """

        curb_ids = ['P01', 'P12', 'P23', 'P30']

        curbs = {}
        for curb_id in curb_ids:
            curbs[curb_id] = curbside.smart_curb(curb_id, 10)

        return curbs
    
    def _init_sim(self, gui):
        """
        initialize simulation in SUMO, accounting for different OS

        @params gui : boolean, whether to run the simulation with gui
        @returns : simulation connection labelled by user
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
        """
        simulate for the defined window length
        """
            
        for _ in range(self.WINDOW):
            
            self.sim.simulationStep()

            self.TIME += 1

            self._micro()
            
    def _micro(self):
        """
        control vehicle parking movements microscopically
        this serves as the physical maneuvers behind the scene

        that said, even if our controllers are curbs, we still need to define our vehicles park

        presumably, this micro control is conducted at the finest time granularity

        @params 
        @returns
        """
        
        for curb in self.curbs.values():
            # obtain vehicles that are already parked or accepted but not yet arrived
            curb.parked_veh = self.sim.parkingarea.getVehicleIDs(curb.id)
            # those who have arrived are not in accepted mode any more
            curb.accepted_veh = curb.accepted_veh.difference(curb.parked_veh)

            # update occupancy for self and self's record of neighbors
            # note accepted vehicles are also occupying spaces even if they have not arrived
            curb._occupy_cnt()
            curb._update_neigh_occupy(self.curbs)
            
            # collect vehicles that arrived at the induction loop at the edge
            v_enter = set([item[0] for item in self.sim.inductionloop.getVehicleData("L" + curb.id[1:])])
            
            for veh in v_enter:
                
                # vehicle entering can be of 3 types (b, ncv, cv)
                # first filter on those who actually are parking at this lane
                # for those passers-by, do nothing
                if not veh.startswith('b', 0, 1)\
                   and self.sim.vehicle.getNextStops(veh) \
                   and curb.id in [item[2] for item in self.sim.vehicle.getNextStops(veh)] \
                   and veh not in curb.accepted_veh and veh not in curb.parked_veh:
                   
                   # notice last two filter make sure the vehicle have not been considered
                   # if the vehicle has been rejected, then this curb should not be in the stop list
                   # if the vehicle has been accepted, then the filter filters it out
                        
                    # vclass: passenger or delivery
                    vclass = self.sim.vehicle.getVehicleClass(veh)
                    if vclass == 'delivery':
                        occ = curb.cv_occ
                        cap = curb.cv_cap
                    else:
                        occ = curb.ncv_occ
                        cap = curb.ncv_cap
                        
                    if occ < cap: # if allowed to park physically
                        
                        # add to accepted list
                        curb.accepted_veh.add(veh)
                        # update curb occ and cap immediately when a vehicle is accepted
                        # this is multiple vehicles can enter in the same second
                        curb._occupy_cnt()

                    else: # not enough space at this curb

                        # produce reroute suggestion
                        reroute_curb = curb._reroute_choice(veh, self.curbs)
                        
                        # change destination to next exit
                        self.sim.vehicle.changeTarget(veh, "E" + reroute_curb[1:])

                        # reroute to neighbor curb
                        self.sim.vehicle.rerouteParkingArea(veh, reroute_curb)

                        # highlight in gui mode
                        if self.GUI:
                            self.sim.vehicle.highlight(veh, color=(255, 0, 255), size=10)
                            
            
    def _get_state(self):
        """
        retrieve state of each curb: pressure, cv_occ, ncv_occ, cv_cap, ncv_cap

        pressure: borrowed from traffic signal control, i.e., cumulative/average delay
        """
        
        state = []
        
        for i in ['P01', 'P12', 'P23', 'P30']:
            curb = self.curbs[i]

            # obtain vehicle from current previous curb edge
            # note in simple cases, vehicles on other edges cannot be considering parking at this curb
            veh_curr_edge = self.sim.edge.getLastStepVehicleIDs(curb.edge)
            veh_prev_edge = self.sim.edge.getLastStepVehicleIDs("E" + str((int(curb.edge[1]) + 3) % 4) + curb.edge[1])
            veh_list = veh_curr_edge + veh_prev_edge

            # obtain the list of vehicles planning to park here
            veh_arriving = []
            for veh in veh_list:
                if curb.id in [item[2] for item in self.sim.vehicle.getNextStops(veh)]:
                    veh_arriving.append(veh)

            # obtain driving distance (1) of all parking vehicles, and (2) of only cvs
            full_dist = cv_dist = 0
            full_cnt = cv_cnt = 0
            for veh in veh_arriving:
                if veh.startswith('cv', 0, 2):
                    cv_dist += self.sim.vehicle.getDistance(veh)
                    cv_cnt += 1
                full_dist += self.sim.vehicle.getDistance(veh)
                full_cnt += 1

            # compute avereage driving distance of all and of only cvs
            full_avg_dist = full_dist / full_cnt if full_cnt > 0 else 0
            cv_avg_dist = cv_dist / cv_cnt if cv_cnt > 0 else 0

            # compile state for the curb and append it to state
            tmp = np.array([self.TIME, 
                            # number of vehicles just served?
                            full_cnt,
                            cv_cnt,
                            # average cruising distance by arriving vehicles
                            # critic sees CV + non-CV: cv and ncv
                            full_avg_dist,
                            cv_avg_dist,
                            curb.cv_occ, curb.ncv_occ, curb.cv_cap, curb.ncv_cap])
            
            state.append(tmp)

            # save derived cruising distance to instance for reward calculation later
            curb.full_dist = full_dist
            curb.cv_dist = cv_dist

        return state
    
    def _get_reward(self):
        """
        retrieve reward for the curbs
        """
        reward = []
        # reroute is collected for reward calculation
        for i in ['P01', 'P12', 'P23', 'P30']:
            curb = self.curbs[i]

            # reward is calculated of both CV + non-CV: cv and ncv
            # curb.full_dist, curb.cv_dist
            reward.append((curb.full_dist, curb.cv_dist))

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

        
        @params actions : dictionary, actions associated with each curb agent, computed outside of the env

        @returns state : dictionary, state of each curb agent
        @returns reward : dictionary, reward of each curb agent
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
        """
        end simulation
        """
        traci.close()
        self.close()


