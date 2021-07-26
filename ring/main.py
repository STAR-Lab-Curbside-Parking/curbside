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
    
    utils.generate_route(park2curb=0.8, background2park=1, cv2ncv_pf=1, cv2ncv_pd=0.5)
    env = envs.RingEnv(NET_XML, ADD_XML, ROU_XML, window=10, gui=True)
    # while True:
    #     env._simulate()
    #     env._get_state()
    #     env._get_reward()

    curb_agents = {k : agent.A2C() for k in ['P01', 'P12', 'P23', 'P30']} 
    s = env._get_state()
    print(s)

    # for epoch in range(10):

    #     res = 0

    #     while env.time_step < 3600:

    #         actions = []

    #         for i in range(len(env.curb_ids)):
    #             agent = curb_agents[env.curb_ids[i]]
    #             policies, _ = agent.forward(s[i])

    #             probs = F.softmax(policies, dim=0)
    #             a = Categorical(probs).sample() - 1

    #             actions.append(a)
            
    #         s_prime, r = env.batch(actions)  
    #         res += sum(r.values())          

    #         for i in range(len(env.curb_ids)):
    #             agent = curb_agents[env.curb_ids[i]]
    #             agent.train(s[i], actions[i], r[env.curb_ids[i]], s_prime[i])

    #         s = s_prime

    #     env.terminate()
    #     time.sleep(2)
    #     env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
    #     s = env._get_state()
        
    #     print('Epoch = {}, global reward = {}'.format(epoch, res))

    # env.terminate()

if __name__ == "__main__":
    main()