'''
curb.py
---------------------------------------------------------------
This file defines curb agents for the curbside management problem.
Thus, it is imported and should create instances there.
---------------------------------------------------------------
python-igraph is documented here: https://igraph.org/python/
graph-tool is documented here (although not used): https://graph-tool.skewed.de/
'''

# import
import os, sys
import heapq
import operator, random

import igraph
import xml.etree.ElementTree as ET

from utils import is_dlv_veh

# general class
class Curbside:
    def __init__(self, add_xml, net_xml, curb_id, vclass):
        """
        initialize instance
        Args:
            add_xml : str, additional file that defines curb spaces in SUMO, "XXX.add.xml"
            net_xml : str, network file that defines simulation network in SUMO, "XXX.net.xml"
            curb_id : str, id of the curb to be created, from user input
            vclass : str, allowable type of vehicle to park at this curb, from user input
        """
        start_dict = {'id': curb_id,
                      'vclass': vclass,
                      'add_xml': add_xml,
                      'net_xml': net_xml}
        curb_add_info = self.read_curb_add(add_xml, curb_id)
        curb_net_info = self.read_curb_net(net_xml, curb_add_info['lane'])

        start_dict = {**start_dict, **curb_add_info, **curb_net_info}

        self.validate(start_dict)

        # static attributes
        self.id = start_dict['id']
        self.lane = start_dict['lane']
        self.edge = self.lane.split('_')[0]
        self.start_pos = start_dict['start_pos']
        self.end_pos = start_dict['end_pos']
        self.width = start_dict['width']
        self.from_junction = start_dict['from_junction']
        self.to_junction = start_dict['to_junction']
        self.length = abs(self.start_pos - self.end_pos)
        self.edge = self.lane.split('_')[0]

        # dynamic attributes
        self.vclass = start_dict['vclass']
        # physically parked vehicle set
        self.parked_vehicle = set()
        # confirmed set: parked + planned (on the way)
        self.occupied_vehicle = set()
        # moving vehicles on the host edge
        self.moving_vehicle = set()

        # internal schedule
        self.schedule = [None] * (24 * 12)
        
        # two capacities
        self.tot_cap = start_dict['capacity']
        self.psg_cap = self.tot_cap
        self.dlv_cap = 0

        # occupancy count
        self.dlv_occ = 0
        self.psg_occ = 0

    @staticmethod
    def read_curb_add(add_xml, curb_id):
        """
        search info for specified curbside/parking area in additional xml in SUMO
        Args:
            add_xml : str, additional document that defines parking areas in SUMO simulation
            curb_id : str, id of curb to query
        Returns:
            result : dictionary
        """
        result = {}
        # read from XML file
        root = ET.parse(add_xml).getroot()

        for child in root.iter('additional'):
            for kid in child.iter('parkingArea'):
                if kid.get('id') == curb_id:
                    # result['name'] = kid.get('name')
                    result['lane'] = kid.get('lane')
                    result['start_pos'] = float(kid.get('startPos'))
                    result['end_pos'] = float(kid.get('endPos'))
                    result['capacity'] = int(kid.get('roadsideCapacity'))
                    result['width'] = int(kid.get('width')) if kid.get('width') is not None else None
                    return result
        raise Exception()

    @staticmethod
    def read_curb_net(net_xml, lane):
        """
        search info for specified lane in additional xml in SUMO. from_ and to_junctions are collected
        will be used in further graph construction and search 
        
        Args:
            net_xml : str, network document in SUMO simulation
            lane : str, id of the lane of the curb
        Returns:
            result : dictionary
        """
        result = {}

        # read from XML file
        root = ET.parse(net_xml).getroot()

        for child in root.iter('edge'):
            if child.get('function') != 'internal':
                for kid in child.iter('lane'):
                    if kid.get('id') == lane:
                        # collect from to junctions of the lane
                        result['from_junction'] = child.get('from')
                        result['to_junction'] = child.get('to')
                        return result
        raise Exception()

    @staticmethod
    def validate(start_dict):
        """
        validate collected info before creating an instance
        
        Args:
            start_dict : dictionary, basic info to be validated
        """
        if start_dict['capacity'] < 0:
            raise Exception()

        if start_dict['width'] is not None and start_dict['width'] < 0:
            raise Exception()

        if start_dict['start_pos'] == start_dict['end_pos']:
            raise Exception()

        if start_dict['vclass'] is None:
            raise Exception()

class SmartCurbside(Curbside):
    """
    smart curb class: a subclass of general curbside
    """
    
    # class property: 
    ## smart (Y/N)
    ## upstream roadnetwork search radius (int)
    ## downstream roadnetwork search radius (int)
    smart = 1
    upstream_search_radius = 1000
    downstream_search_radius = 2000

    def __init__(self, smart, add_xml, net_xml, curb_id, vclass,
                 road_network):
        """
        initialize as its super class does
        """
        super().__init__(add_xml, net_xml, curb_id, vclass)

    def __eq__(self, other):
        return self.id == other.id

    def find_neighborhood(self, road_network, curbs, option='nearest'):
        """
        Find neighborhoods at the vicinity of each smart curb
        only smart curb has this function
        
        Args:
            road_network : igraph instance, the constructued simulation network
            option : string, choice of "vicinity", either "nearest" or "whole"
        
        Returns:
            self.neighbor : initialized dictionary of "(neighbor curb id, distance) - occupancy" pair
        """

        # search
        ## downstream
        def search_downstream(road_network, queue, discovered, output_nodes):
            """
            BFS recursive for downstream implemented with heap queue in Python
            """

            if len(queue) == 0:
                return

            if queue[0][0] > self.downstream_search_radius:
                return

            distance, vertex = heapq.heappop(queue)
            output_nodes.add(vertex)

            for item in road_network.neighbors(vertex, mode="OUT"):
                if not discovered[item]:
                    discovered[item] = True
                    # push to the priority queue
                    heapq.heappush(queue,
                                  (distance + road_network.es.find(_source=vertex, _target=item)['weight'], item))
            # recursive
            search_downstream(road_network, queue, discovered, output_nodes)

        ## upstream
        def search_upstream(road_network, queue, discovered, output_nodes):
            """
            BFS recursive for upstream
            """

            if len(queue) == 0:
                return

            # print(queue[0][0])
            if queue[0][0] > self.upstream_search_radius:
                return

            distance, vertex = heapq.heappop(queue)
            output_nodes.add(vertex)

            for item in road_network.neighbors(vertex, mode="IN"):
                if not discovered[item]:
                    discovered[item] = True
                    # push to the priority queue
                    heapq.heappush(queue, 
                                  (distance+road_network.es.find(_source=item, _target=vertex)['weight'], item))
            # recursive
            search_upstream(road_network, queue, discovered, output_nodes)
        
        # search downstream junctions
        downstream_nodes = set()
        discovered = [False] * len(road_network.vs)
        downstream_queue = []
        heapq.heappush(downstream_queue, (0, road_network.vs.find(name=self.to_junction).index))
        search_downstream(road_network, downstream_queue, discovered, downstream_nodes)

        # search upstream junctions
        upstream_nodes = set()
        discovered = [False] * len(road_network.vs)
        upstream_queue = []
        heapq.heappush(upstream_queue, (0, road_network.vs.find(name=self.from_junction).index))
        search_upstream(road_network, upstream_queue, discovered, upstream_nodes)

        # view can be a radius of items or a local network that can be later embedded
        # this feature can be explored further in the future
        # print(list(downstream_nodes | upstream_nodes))
        self.view = road_network.subgraph(list(downstream_nodes | upstream_nodes))

        # find associated curbs
        self.neighbor = {}
        
        # start_x
        lane_length = self.view.es.find(name=self.edge)['weight']
        start_x = self.start_pos if self.start_pos > 0 else lane_length + self.start_pos

        # shortest path distance and end_x
        # it seems all curbs are included in local view
        for curb_id, curb in curbs.items():
            assert curb_id == curb.id
            if self.id != curb_id:
                if option == "whole":
                    distance = self._roadway_dist(start_x, float(curb.start_pos), curb.edge)
                    self.neighbor[(curb_id, round(distance,2))] = [0, 0]
                else:
                    # nearest
                    if len(set([curb.from_junction, curb.to_junction]).intersection(set([self.from_junction, self.to_junction]))) > 0:
                        distance = self._roadway_dist(start_x, float(curb.start_pos), curb.edge)
                        self.neighbor[(curb_id, round(distance, 2))] = [0, 0]

    def _roadway_dist(self, start_x, dest_pos, target_edge):
        """
        calculates roadway distance between two curbs
        """

        # try:
        #     _ = self.view.es.find(name=target_edge)
        # if the previous statement does not raise error then the following is executed
        destination_node = self.view.es.find(name=target_edge).source

        # shortest path distance
        distance = self.view.shortest_paths_dijkstra(source=self.to_junction,
                                                     target=destination_node, 
                                                     weights='weight', mode='OUT')[0][0]

        # end_x from the last junction to destination curb
        if dest_pos >= 0:
            end_x = dest_pos
        else:
            end_x = self.view.es.find(name=target_edge)['weight'] + dest_pos

        # adjust for first and last mile
        if (self.view.es.find(name=self.edge).source == \
            self.view.vs.find(name=self.from_junction).index) and (end_x > start_x):
            distance = end_x - start_x
        else:
            distance += end_x + (self.view.es.find(name=self.edge)['weight'] - start_x)
        
        return distance

    def _update_neigh_occupy(self, curbs):
        """
        update nearby_cub status
        """

        # update neighbor delivery + passenger occupancy
        for curb_pair in self.neighbor:
            assert len(curbs[curb_pair[0]].occupied_vehicle) == curbs[curb_pair[0]].dlv_occ + curbs[curb_pair[0]].psg_occ
            self.neighbor[curb_pair] = [curbs[curb_pair[0]].dlv_occ, curbs[curb_pair[0]].psg_occ]

    # generate reroute suggestion
    # in the future, this can be extended to estimates of nearby curbs
    def _reroute_choice(self, veh_id, curbs):
        """
        based on current status of nearby curbs
        make a reroute decision
        """ 

        # if delivery vehicle, only look at delivery vehicle occupancy
        if is_dlv_veh(veh_id):
            available_curbs = [curb_pair for curb_pair in self.neighbor if self.neighbor[curb_pair][0] < curbs[curb_pair[0]].dlv_cap]
        else:
            available_curbs = [curb_pair for curb_pair in self.neighbor if self.neighbor[curb_pair][1] < curbs[curb_pair[0]].psg_cap]

        if len(available_curbs) > 0:
            # if there is available neighbor of specified type: delivery/ passenger
            available_curbs.sort(key=operator.itemgetter(1))

            # return closest available curb and associated distance
            return available_curbs[0] 

        else:
            # if no available spots of requested type in neighbors, return a random one of them
            return random.choice(list(self.neighbor.keys()))
    
    def _occupy_cnt(self):
        """
        update occupancy of two types of vehicles for a curb
        """
        # devliery vehicle count
        self.dlv_occ = len([veh for veh in self.occupied_vehicle if is_dlv_veh(veh)])
        
        # passenger vehicle count
        self.psg_occ = len(self.occupied_vehicle) - self.dlv_occ

