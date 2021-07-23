from itertools import product
import numpy as np
import time

import curbside
import utils
import envs
import generator

NET_XML = "seattle.net.xml"
ADD_XML = "seattle.add.xml"
ROU_XML = "seattle.trips.xml"

env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
policy = generator.Policy(env.curb_ids, policy_name='nn')
state = env._get_state()

res = {}
res['rewards'] = []
res['total_reroute'] = []
res['total_failed'] = []

for epoch in range(2):
    while True:
        actions = policy.forward(state)
        next_state, reward = env.batch(actions)
        res['rewards'].append(sum(reward.values()))
        res['total_reroute'].append(env.reroute_total)
        res['total_failed'].append(env.failed_total)

        state = next_state

        for i in range(len(policy.curb_ids)):
            policy.agent.optimize(reward[policy.curb_ids[i]])

        if env.time_step >= 3600:
            env.terminate()
            time.sleep(2)
            env = envs.SeattleEnv(NET_XML, ADD_XML, ROU_XML, dlv_prc=5, psg_prc=1, gui=False)
            state = env._get_state()
            # env.sim = env._init_sim(env.gui)
            # state = env.reset()
            break
    
    print('Epoch = {}, global reward = {}'.format(epoch, np.cumsum(res['rewards'])[60*epoch+59] - np.cumsum(res['rewards'])[60*epoch]))

env.terminate()