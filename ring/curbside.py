'''
curb.py
---------------------------------------------------------------
This file defines curb agents for the curbside management problem.
Thus, it is imported and should create instances there.

'''

# import

from utils import is_cv_veh

class smart_curb:
    """
    smart curb class: a subclass of general curbside
    """

    def __init__(self, curb_id, capacity=10):
        """
        initialize instance
        Args:
            add_xml : str, additional file that defines curb spaces in SUMO, "XXX.add.xml"
            net_xml : str, network file that defines simulation network in SUMO, "XXX.net.xml"
            curb_id : str, id of the curb to be created, from user input
        """

        # static attributes
        self.id = curb_id
        self.edge = "E" + self.id[1:]
        self.start_pos = 35

        # downstream neighbor
        self.neighbor = {"P" + self.id[-1] + str((int(self.id[1]) + 2) % 4): [0,0,0,0]}
        
        # two capacities
        self.tot_cap = capacity
        self.ncv_cap = self.tot_cap - 1
        self.cv_cap = 1

        # occupancy count
        self.cv_occ = 0
        self.ncv_occ = 0

        # physically parked vehicle set
        self.parked_veh = set()

    def __eq__(self, other):
        return self.id == other.id

    def _update_neigh_occupy(self, curbs):
        """
        update nearby_cub status
        """

        # update neighbor occupancies and capacities
        for key in self.neighbor:
            self.neighbor[key] = [curbs[key].cv_occ, curbs[key].ncv_occ, 
                                  curbs[key].cv_cap, curbs[key].ncv_cap]

    def _reroute_choice(self, veh_id, curbs):
        """
        offer reroute suggestion, always return the immediate downstream in the ring setting
        """ 

        return list(self.neighbor.keys())[0]
    
    def _occupy_cnt(self):
        """
        update occupancy of two types of vehicles for a curb
        """
        # devliery vehicle count
        self.cv_occ = len([veh for veh in self.parked_veh if is_cv_veh(veh)])
        
        # passenger vehicle count
        self.ncv_occ = len(self.parked_veh) - self.cv_occ

