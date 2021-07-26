import numpy as np
import random, time
import torch
import torch.nn.functional as F
from torch.distributions import Categorical

import agent, curbside, envs, utils

random.seed(0)

NET_XML = "ring.net.xml"
ADD_XML = "ring.add.xml"
ROU_XML = "ring.trips.xml"

def main():

    id_list = ['P01', 'P12', 'P23', 'P30']
    
    utils.generate_route(park2curb=0.8, background2park=1, cv2ncv_pf=1, cv2ncv_pd=0.5)
    env = envs.RingEnv(NET_XML, ADD_XML, ROU_XML, window=10, gui=True)

    curb_agents = {k : agent.A2C() for k in id_list} 
    s = env._get_state()

    for epoch in range(10):

        res = 0

        while env.sim.simulation.getMinExpectedNumber() > 0:

            if env.TIME % env.WINDOW == 0:

                actions = []

                for i in range(len(id_list)):
                    id = id_list[i]
                    policies, _ = curb_agents[id].forward(s[i])

                    probs = F.softmax(policies, dim=0)
                    a = Categorical(probs).sample() - 1

                    actions.append(a)
                print(actions)
                # state is vector of 9 elements: time, full_cnt, cv_cnt, fll_avg_dist, cv_avg_dist, cv_occ, ncv_occ, cv_cap, ncv_cap
                # reward is tuple of 2 elements: full_dist, cv_dist
                # depends on agent design, can use different elements for training

                s_prime, r = env.batch(actions)  
                res += sum([item[0] for item in r])          

                for i in range(len(id_list)):
                    curb_agents[id_list[i]].train(s[i], actions[i], r[i], s_prime[i])

                s = s_prime

        env.terminate()
        time.sleep(2)

        # reset
        env = envs.RingEnv(NET_XML, ADD_XML, ROU_XML, window=10, gui=True)
        s = env._get_state()
        
        print('Epoch = {}, global reward = {}'.format(epoch, res))

    env.terminate()

if __name__ == "__main__":
    main()