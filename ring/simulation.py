
import numpy as np
import random
import timeit
import torch
import torch.nn.functional as F
from torch.distributions import Categorical

import envs, utils

torch.manual_seed(0)
np.random.seed(0)
random.seed(0)

NET_XML = "ring.net.xml"
ADD_XML = "ring.add.xml"
ROU_XML = "ring.trips.xml"


class Simulation:
    def __init__(self, Model, Memory, training_epochs):
        self._Model = Model
        self._Memory = Memory

        self._training_epochs = training_epochs

    def run(self, episode):
        """
        Runs an episode of simulation, then trains
        """
        start_time = timeit.default_timer()

        id_list = ['P01', 'P12', 'P23', 'P30']

        GUI = False
        # if episode == 2:
        #     GUI = True
        env = envs.RingEnv(NET_XML, ADD_XML, ROU_XML, window=30, gui=GUI)
        early_stop = False

        old_s = env._get_state()
        old_a = 0

        res = 0
        while env.sim.simulation.getMinExpectedNumber() > 0:

            # state is list of 8 elements: full_arrival, cv_arrival, full_avg_reroute, cv_avg_reroute, cv_occ, ncv_occ, cv_cap, ncv_cap
            # reward is tuple of 2 elements: full_reroute, cv_reroute

            old_a = 0
            new_s, r = env.batch(old_a)
            reward = utils.interpret_reward(r)

            # penalize for creating congestion
            if sum([env.sim.edge.getLastStepOccupancy(env.curbs[id].edge)>0.62 for id in env.curbs.keys()]) > 3\
                or sum([env.sim.edge.getWaitingTime(id)>300 for id in ['A0', 'A1', 'A2', 'A3']]) > 3:
                reward = -100000
                early_stop = True
            
            # penalize beyond-boundary attempts
            # cap
            # if sum([env.curbs[i].cv_cap + old_a < env.curbs[i].cv_occ or env.curbs[i].ncv_cap + old_a < env.curbs[i].ncv_occ \
            #     for i in ['P01', 'P12', 'P23', 'P30']]) > 0:
            #     reward = -1000

            res += reward
            self._Memory.add_sample((old_s, old_a, reward, new_s))

            new_a = self._choose_action(new_s)

            old_s = new_s
            old_a = new_a

            if early_stop:
                print("full_arrival", "cv_arrival", "full_avg_reroute", "cv_avg_reroute", "cv_occ", "ncv_occ", "cv_cap", "ncv_cap")
                print(new_s)
                break

        env.terminate()

        print("Total reward:", res)
        simulation_time = round(timeit.default_timer() - start_time, 1)

        print("Training...")
        start_time = timeit.default_timer()

        # self._replay()

        training_time = round(timeit.default_timer() - start_time, 1)
        return simulation_time, training_time

    def _choose_action(self, state):

        # actor only uses cv_state
        cv_state = utils.interpret_state(state)[1]

        s = torch.FloatTensor(cv_state).to(self._Model.device)
        policy = self._Model.actor(s)
        probs = F.softmax(policy, dim=0)
        # print(probs.data)
        a = Categorical(probs).sample().cpu().detach().numpy() - 1
        return a

    def _replay(self):

        """
        Retrieve a group of samples from the memory and for each of them update the learning equation, then train

        replay buffer refer to
        https://github.com/SafeguardLi/DRL-for-Traffic-Control_wz/blob/6de45ef47a3876bfc41de9be7885e4a5fc49329f/Single%20Intersection/training_simulation.py

        one big difference between Pytorch dataloader and manual sampling is that dataloader loads the complete dataset for sure
        This may be true as well for manual sampoling, but I just haven't had the cahnce to do that
        training in batches could be more efficient
        """

        losses = []

        train_data = utils.memory_dataset(self._Memory._samples, utils.transform())

        params = {
            'batch_size': self._Model.batch_size,
            'sampler': torch.utils.data.RandomSampler(train_data),
            # 'shuffle': True,
            'num_workers': 2,
            'drop_last': True,
            'pin_memory': True
        }

        train_data_gen = torch.utils.data.DataLoader(train_data, **params)

        for i in range(self._training_epochs):
            loss = 0

            for local_batch in train_data_gen:
                # local_bath[0][0] is full_s, a list of length 6, each of which is tensor of 64

                loss += self._Model.train(local_batch, True)
            
            losses.append(loss)

        # if i * self._Model.batch_size <= self._Memory._size_now() - self._Model.batch_size:
        #     batch = self._Memory._samples[i*self._Model.batch_size: (i+1)*self._Model.batch_size]

        #     for _, b in enumerate(batch):
        #         self._Model.train(b)
        