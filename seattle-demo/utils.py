import random

import igraph
import pandas as pd
import xml.etree.ElementTree as ET

def create_graph(net_xml):
    road_network = igraph.Graph(directed=True)

    root = ET.parse(net_xml).getroot()

    # add vertices with names
    # can refer to nodes with their names, instead of id's in most cases
    for child in root.iter('junction'):
        _ = road_network.add_vertex(name=child.get('id'))
    for child in root.iter('edge'):
        if child.get('function') != 'internal':
            _ = road_network.add_edge(child.get('from'), child.get('to'),
                                      name=child.get('id'),
                                      weight=float([kid.get('length') for kid in child.iter('lane')][0]))

    return road_network  

def generate_route(psg_park_dm_x=1, 
                   dlv_park_dm_x=1, dlv_park_time=3*60, dlv_num=100, 
                   flow_num=200, 
                   net_xml = "seattle.net.xml", add_xml = "seattle.add.xml", rou_xml = "seattle.trips.xml",
                   demand_csv = "parking_demand_dist.csv",
                   simulate_hour=12, seed=2):
    """
    generate route file for simulation

    Args:
        psg_park_dm_x : float, passenger veh parking demand scaleing
        dlv_park_dm_x : float, delivery veh parking demand scaling
        dlv_park_time : int, park duration of each delivery veh
        dlv_num : int, base count of parking delivery veh, to be multiplied by dlv_park_dm_x
        simulate_hour : int, the hour of day to be simulated, default 12-1pm
        seed : int, random seed

    Returns:
        None (results printed to route file)
    """
    
    random.seed(seed)
    
    # get parking area info, including parkingArea IDs and capaciy
    parking_areas = []
    parking_areas_capacity = []
    root_add = ET.parse(add_xml).getroot()
    for child in root_add.iter('parkingArea'):
        parking_areas.append(child.get('id'))
        parking_areas_capacity.append(child.get('roadsideCapacity'))
    parking_areas_capacity = list(map(int, parking_areas_capacity)) 

    # get all network edges
    edges_all = []
    root_net = ET.parse(net_xml).getroot()
    for child in root_net.iter('edge'):
        if child.get('function') != 'internal':
            edges_all.append(child.get('id'))

    # get edges can be origin and destination of trips
    edges_od = []
    edges_od.extend([i for i in edges_all if i[3:5] == '00'])
    edges_od.extend([i for i in edges_all if i[3:5] == '04'])
    edges_od.extend([i for i in edges_all if i[3:5] == '23'])
    edges_od.extend([i for i in edges_all if i[3:5] == '27'])

    edges_o = ['01-23-NW', '25-00-NE', '26-00-NE', '27-00-NE', '01-27-SE', '02-27-SE', 
               '03-27-SE', '27-04-SW', '04-23-NW', '03-23-NW']
    edges_d = [i for i in edges_od if i not in edges_o]

    # get parking demand from historical parking transactions 
    parking_demand = pd.read_csv(demand_csv)

    # generate parking demand and host in list
    ## later will be sorted by depart time
    parking_leq_60min_num = parking_demand[str(simulate_hour-1)].values[3] + parking_demand[str(simulate_hour-1)].values[4] + parking_demand[str(simulate_hour-2)].values[5] + parking_demand[str(simulate_hour-4)].values[6]
    parking_geq_60min_num = parking_demand[str(simulate_hour-1)].values[5] + parking_demand[str(simulate_hour-1)].values[6]+ parking_demand[str(simulate_hour-2)].values[6] + parking_demand[str(simulate_hour-3)].values[6]

    parking_availble_dict = dict(zip(parking_areas, parking_areas_capacity))
    parking_areas_availble = parking_areas.copy()

    parking_10min_num = parking_demand[str(simulate_hour)].values[0]
    parking_20min_num = parking_demand[str(simulate_hour)].values[1]
    parking_30min_num = parking_demand[str(simulate_hour)].values[2]
    parking_60min_num = parking_demand[str(simulate_hour)].values[3] \
                        + parking_demand[str(simulate_hour)].values[4] \
                        + parking_demand[str(simulate_hour)].values[6] 

    parking_10min_num *= psg_park_dm_x
    parking_20min_num *= psg_park_dm_x
    parking_30min_num *= psg_park_dm_x
    parking_60min_num *= psg_park_dm_x

    parking_config_list = []

    for i in range(int(parking_10min_num)):
        pa = random.choice(parking_areas) # parking area
        o = random.choice(edges_o)
        d = random.choice(edges_d)
        dt = random.randint(1, 3600) # depart time
        pt = 10 * 60 # parking time
        parking_config_list.append((dt, o, d, pa, pt, "psg"))

    for i in range(int(parking_20min_num)):
        pa = random.choice(parking_areas) # parking area
        o = random.choice(edges_o)
        d = random.choice(edges_d)
        dt = random.randint(1, 3600) # depart time
        pt = 20 * 60 # parking time
        parking_config_list.append((dt, o, d, pa, pt, "psg"))

    for i in range(int(parking_30min_num)):
        pa = random.choice(parking_areas) # parking area
        o = random.choice(edges_o)
        d = random.choice(edges_d)
        dt = random.randint(1, 3600)  # depart time
        pt = 30 * 60 # parking time
        parking_config_list.append((dt, o, d, pa, pt, "psg"))

    for i in range(int(parking_60min_num)):
        pa = random.choice(parking_areas) # parking area
        o = random.choice(edges_o)
        d = random.choice(edges_d)
        dt = random.randint(1, 3600)  # depart time
        pt = 60 * 60 # parking time
        parking_config_list.append((dt, o, d, pa, pt, "psg"))


    dlv_num *= dlv_park_dm_x

    for i in range(int(dlv_num)):
        pa = random.choice(parking_areas) # parking area
        o = random.choice(edges_o)
        d = random.choice(edges_d)
        dt = random.randint(1, 3600) # depart time
        pt = dlv_park_time
        parking_config_list.append((dt, o, d, pa, pt, "dlv"))

    parking_config_list.sort(key=lambda tup: tup[0])  # sorts in place

    # start to generate route file
    with open(rou_xml, "w") as rou:
        # write headers
        print("""<?xml version="1.0" encoding="UTF-8"?>
              <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
              """, 
              file=rou)

        print("""\t<vType id="psg" vClass="passenger" guiShape="passenger/sedan" color="255,183,59"></vType>\n
    <vType id="pass" vClass="passenger" guiShape="passenger/hatchback" color="192,192,192"></vType>\n
    <vType id="dlv" vClass="delivery" guiShape="truck" color="245,39,224" length="6.0" width="3"></vType>
          """, 
          file=rou)

        # write background flow
        for i in range(len(edges_o)):
            o = edges_o[i]
            d = [item for item in edges_d if item[:2] == o[:2] and item[:5] != o[:5]]
            if len(d) >= 1:
                d = d[0]
            else:
                d = [item for item in edges_d if item[:2] == str(int(o[:2])+1)][0]
            print('\t<flow id="f{}" begin="{}" end="{}" number="{}" from="{}" to="{}" type="pass"/>'.format(
                                                                                               i+200, 0, 3600, 
                                                                                               flow_num, 
                                                                                               o, d), 
                  file=rou)

        # write existing vehicles at the start of simulation
        ## these started prior to 12pm and park longer than 60min
        for i in range(parking_geq_60min_num):
            parking_area = random.choice(parking_areas_availble) # parking area
            parking_availble_dict[parking_area] -= 1
            if parking_availble_dict[parking_area] == 0:
                parking_areas_availble.remove(parking_area)
            d = random.choice(edges_d)
            parking_left_time = 3600
            print('''\t<trip id="exist_g60_{}" depart="{}" to="{}" type="psg">
        <stop index="0" parkingArea="{}" duration="{}" parking="true"/>
    </trip>'''.format(i, 0, d, parking_area, parking_left_time), file = rou)
        
        ## these started prior to 12pm and park shorter than 60min
        for i in range(parking_leq_60min_num):
            parking_area = random.choice(parking_areas_availble) # parking area
            parking_availble_dict[parking_area] -= 1
            if parking_availble_dict[parking_area] == 0:
                parking_areas_availble.remove(parking_area)
            d = random.choice(edges_d)
            parking_left_time = (60 - random.randint(1, 60)) * 60
            print('''\t<trip id="exist_l60_{}" depart="{}" to="{}" type="psg">
        <stop index="0" parkingArea="{}" duration="{}" parking="true"/>
    </trip>'''.format(i, 0, d, parking_area, parking_left_time), file = rou)
    
        # write generated parking
        ## including passenger veh and dlvs
        for i in range(len(parking_config_list)):
            dt, o, d, pa, pt, vType = parking_config_list[i]
            print('''\t<trip id="{}_{}" depart="{}" from="{}" to="{}" type="{}">
        <stop index="0" parkingArea="{}" duration="{}" parking="true"/>
    </trip>'''.format(vType, i, dt, o, d, vType, pa, pt), file = rou)

        # write end of file
        print('\n</routes>', file = rou)

#     print(rou_xml, 're-generated. (demand factor =', psg_park_dm_x, ')')


def is_dlv_veh(veh):
    return veh.startswith('dlv', 0, 3)
    # return traci.vehicle.getVehicleClass(veh) == 'delivery'