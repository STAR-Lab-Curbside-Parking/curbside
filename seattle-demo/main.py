from itertools import product
import numpy as np
import time
import torch
import torch.nn.functional as F
from torch.distributions import Categorical
from torch.autograd import Variable

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
    env.control_window = 60
    curb_agents = {k:generator.AC() for k in env.curb_ids} 
    s = env._get_state()

    ep_reward = [0.] * 20

    for epoch in range(20):
        while env.time_step < 1800:

            actions = []

            for i in range(len(env.curb_ids)):
                agent = curb_agents[env.curb_ids[i]]
                # print(s[i])
                policies, _ = agent.forward(s[i])

                # samples from categorical distribution is [0,...,K-1]
                # our action space is [-1, 0, 1]
                probs = F.softmax(policies, dim=0)
                a = Categorical(probs).sample() - 1

                actions.append(a)
            
            s_prime, r = env.batch(actions)  
            ep_reward[epoch] += sum(r.values())          

            for i in range(len(env.curb_ids)):
                agent = curb_agents[env.curb_ids[i]]
                
                policies, value = agent.forward(s[i])
                _, v_prime = agent.forward(s_prime[i])

                td_target = agent.compute_target(v_prime, r[env.curb_ids[i]])

                advantage = value - td_target

                log_probs = F.log_softmax(policies, dim=0)
                log_action_probs = log_probs.gather(0, Variable(actions[i] + 1))

                policy_loss = (-log_action_probs * Variable(torch.tensor(advantage))).sum()
                value_loss = (.5 * (value - Variable(torch.tensor(r[env.curb_ids[i]]))) ** 2.).sum()
                
                loss = policy_loss + value_loss
                loss.backward()
                
                agent.optimizer.step()
                agent.optimizer.zero_grad()

            s = s_prime

        env.terminate()
        time.sleep(2)
        env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
        env.control_window = 60
        s = env._get_state()
        
        print('Epoch = {}, global reward = {}'.format(epoch, np.sum(ep_reward[epoch])))

    env.terminate()

if __name__ == "__main__":
    main()