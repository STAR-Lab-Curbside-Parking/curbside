import random
import pandas as pd
import xml.etree.ElementTree as ET
import torch
from torch.utils.data import Dataset

random.seed(0)

def generate_route(length,
                   park2curb, background2park, cv2ncv_pf, cv2ncv_pd,
                   demand_curve='flat',
                   route_xml='ring.rou.xml'):
    """
    generate route file for simulation

    @params length : int, length of simulation
    @params env : gym env, environment object that has been created and initiated
    @params park2curb : float, ratio of parking traffic to curb space capacities
    @params background2park : float, ratio of background to parking traffic
    @params cv2ncv_pf : float, ratio of cv to non-cv in parking flow
    @params cv2ncv_pd : float, ratio of cv to non-cv in parking duration
    @params route_xml : string, route file to write to

    @return 
    """
    
    random.seed(0)

    park_record = []
    ncv_pd = 60 * 3
    cv_pd = int(ncv_pd * cv2ncv_pd)

    for curb_id in ['P01', 'P12', 'P23', 'P30']:
        o = "A" + curb_id[1]
        d = "E" + curb_id[1:]

        # need to implement demand curve
        park_demand = 10 * length / max(ncv_pd, cv_pd) * park2curb
        background_flow = int(park_demand * background2park)

        cv_demand = park_demand * cv2ncv_pf / (1 + cv2ncv_pf)
        ncv_demand = park_demand * 1 / (1 + cv2ncv_pf)

        for j in range(int(cv_demand)):
            dt = random.randint(100, length+100)
            pt = cv_pd
            park_record.append((dt, curb_id, o, d, pt, 'cv'))

        for j in range(int(ncv_demand)):
            dt = random.randint(100, length+100)
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
        for i in range(4):
            print('\t<flow id="back_{}" begin="{}" end="{}" number="{}" from="{}" to="{}" type="f"/>'.\
                format(i, 0, 3700, background_flow, "A{}".format(i), "E{}{}".format((i + 4 - 1)%4, i)), 
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
    """
    detemines if vehicle is cv by its id

    @params veh : str, veh id
    @ return : if the veh is cv or not
    """
    return veh.startswith('cv', 0, 2)
    # return traci.vehicle.getVehicleClass(veh) == 'delivery'

def interpret_state(s):
    """
    interpret state returned by simulation environment
    
    @params s : list of state elements returned by simulation environment

    @returns full_state, cv_state
    """
    full_state = [sum([item[0] for item in s]), sum([item[2] for item in s]), 
                       sum([item[4] for item in s]), sum([item[5] for item in s]), 
                       sum([item[6] for item in s]), sum([item[7] for item in s])]
    cv_state = [sum([item[1] for item in s]), sum([item[3] for item in s]), 
                sum([item[4] for item in s]), sum([item[5] for item in s]), 
                sum([item[6] for item in s]), sum([item[7] for item in s])]
    
    return full_state, cv_state

def interpret_reward(r):
    """
    interpret reward returned by simulation environment

    @params r : tuple of elements, reward returned by simulation environment

    @returns reward : full reward including both cv and non-cv
    """

    reward = sum([item[0] for item in r])

    return reward

class transform(object):

    def __call__(self, sample):
        """
        transform
        """
        res = []

        # state, action, reward, new_state
        res.append(interpret_state(sample[0]))
        res.append(sample[1])

        # reward is already interpreted outside before
        # res.append(interpret_reward(sample[2]))
        res.append(sample[2])
        res.append(interpret_state(sample[3]))

        return res

class memory_dataset(Dataset):
    def __init__(self, data, transform=None):
        """
        @params samples: list
        @params transform: Optional transform to be applied on a sample.
        """
        self.data = data
        self.transform = transform

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        sample = self.data[idx] # state, action, reward, new_state

        if self.transform is not None:
            sample = self.transform(sample)

        return sample