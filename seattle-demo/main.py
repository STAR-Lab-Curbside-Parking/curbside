from itertools import product
import numpy as np
import time
import torch
import torch.nn.functional as F
from torch.distributions import Categorical

import curbside
import utils
import envs
import generator

NET_XML = "seattle.net.xml"
ADD_XML = "seattle.add.xml"
ROU_XML = "seattle.trips.xml"

np.random.seed(0)


def main():
    env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
    curb_agents = {k : generator.A2C() for k in env.curb_ids} 
    s = env._get_state()

    for epoch in range(10):

        res = 0

        while env.time_step < 1800:

            actions = []

            for i in range(len(env.curb_ids)):
                agent = curb_agents[env.curb_ids[i]]
                policies, _ = agent.forward(s[i])

                probs = F.softmax(policies, dim=0)
                a = Categorical(probs).sample() - 1

                actions.append(a)
            
            s_prime, r = env.batch(actions)  
            res += sum(r.values())          

            for i in range(len(env.curb_ids)):
                agent = curb_agents[env.curb_ids[i]]
                agent.train(s[i], actions[i], r[env.curb_ids[i]], s_prime[i])

            s = s_prime

        env.terminate()
        time.sleep(2)
        env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
        s = env._get_state()
        
        print('Epoch = {}, global reward = {}'.format(epoch, res))

    env.terminate()

if __name__ == "__main__":
    main()