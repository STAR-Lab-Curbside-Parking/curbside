import random
import pandas as pd
import xml.etree.ElementTree as ET

def generate_route(park2curb, background2park, cv2ncv_pf, cv2ncv_pd,
                   demand_curve='flat',
                   route_xml='ring.rou.xml'):
    """
    generate route file for simulation

    @params env : gym env, environment object that has been created and initiated
    @params park2curb : float, ratio of parking traffic to curb space capacities
    @params background2park : float, ratio of background to parking traffic
    @params cv2ncv_pf : float, ratio of cv to non-cv in parking flow
    @params cv2ncv_pd : float, ratio of cv to non-cv in parking duration
    @params route_xml : string, route file to write to

    Returns:
        None (results printed to route file)
    """
    
    random.seed(0)

    park_record = []
    ncv_pd = 60 * 2
    cv_pd = int(ncv_pd * cv2ncv_pd)

    for curb_id in ['P01', 'P12', 'P23', 'P30']:
        o = "A" + curb_id[1]
        d = "E" + curb_id[1:]

        # need to implement demand curve
        park_demand = 10 * 3600 / max(ncv_pd, cv_pd) * park2curb
        background_flow = int(park_demand * background2park)

        cv_demand = park_demand * cv2ncv_pf / (1 + cv2ncv_pf)
        ncv_demand = park_demand * 1 / (1 + cv2ncv_pf)

        for j in range(int(cv_demand)):
            dt = random.randint(100, 3700)
            pt = cv_pd
            park_record.append((dt, curb_id, o, d, pt, 'cv'))

        for j in range(int(ncv_demand)):
            dt = random.randint(100, 3700)
            pt = ncv_pd
            park_record.append((dt, curb_id, o, d, pt, 'ncv'))

    park_record.sort(key=lambda tup: tup[0])  # sorts in place

    # start to generate route file
    with open(route_xml, "w") as rou:
        # write headers
        print("""<?xml version="1.0" encoding="UTF-8"?>
              <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
              """, 
              file=rou)

        print("""\t<vType id="ncv" vClass="passenger" guiShape="passenger/sedan" color="255,183,59"></vType>\n
    <vType id="f" vClass="passenger" guiShape="passenger/hatchback" color="192,192,192"></vType>\n
    <vType id="cv" vClass="passenger" guiShape="evehicle" color="245,39,224"></vType>
          """, 
          file=rou)

        # write background flow
        print('\t<flow id="back" begin="{}" end="{}" number="{}" from="{}" to="{}" type="f"/>'.\
              format(0, 3700, background_flow, "E01", "E30"), 
              file=rou)
    
        # write generated parking
        for i in range(len(park_record)):
            dt, curb_id, o, d, pt, vType = park_record[i]
            print('''\t<trip id="{}_{}" depart="{}" from="{}" to="{}" type="{}">
        <stop index="0" parkingArea="{}" duration="{}" parking="true"/>
    </trip>'''.format(vType, i, dt, o, d, vType, curb_id, pt), file = rou)

        # write end of file
        print('\n</routes>', file = rou)


def is_cv_veh(veh):
    return veh.startswith('cv', 0, 2)
    # return traci.vehicle.getVehicleClass(veh) == 'delivery'