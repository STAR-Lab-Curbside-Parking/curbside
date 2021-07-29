'''
agent.py
---------------------------------------------------------------
This file defines controllers for each curb

Check out the following for building A() and C() separately
https://github.com/MrSyee/pg-is-all-you-need/blob/master/01.A2C.ipynb
'''
import numpy as np
import random
import torch
import torch.nn as nn
from torch import optim
import torch.nn.functional as F
from torch.distributions import Categorical

import utils

torch.manual_seed(0)
np.random.seed(0)
random.seed(0)

def init(module, weight_init, bias_init, gain=1):
    weight_init(module.weight.data, gain=gain)
    bias_init(module.bias.data)
    return module

init_layer = lambda m: init(m, nn.init.orthogonal_, lambda x: nn.init.constant_(x, 0), \
             nn.init.calculate_gain('relu'))

class Actor(nn.Module):
    def __init__(self, in_dim, out_dim, width, num_layer):
        """Initialize."""
        super(Actor, self).__init__()
        
        self.net = nn.Sequential(
            init_layer(nn.Linear(in_dim, width)), nn.ReLU(),
            init_layer(nn.Linear(width, width)), nn.ReLU(),
            # init_layer(nn.Linear(width, width)), nn.ReLU(),
            nn.Linear(width, out_dim)
        )

    def forward(self, state):
        """
        state here must be interpreted list
        """
        pi_out = self.net(state)
        
        return pi_out
    
class Critic(nn.Module):
    def __init__(self, in_dim, width, num_layer):
        """Initialize."""
        super(Critic, self).__init__()
        
        self.net = nn.Sequential(
            init_layer(nn.Linear(in_dim, width)), nn.ReLU(),
            init_layer(nn.Linear(width, width)), nn.ReLU(),
            # init_layer(nn.Linear(width, width)), nn.ReLU(),
            nn.Linear(width, 1)
        )

    def forward(self, state):
        """Forward method implementation."""
        v_out = self.net(state)
        
        return v_out

class A2C:
    """
    initialize instance
    """
    def __init__(self, gamma, learning_rate, batch_size=64):
        super(A2C, self).__init__()

        self.gamma = gamma
        self.learning_rate = learning_rate
        self.batch_size = batch_size

        self.entropy_weight = 0.8

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.actor = Actor(in_dim=6, out_dim=3, width=2*6, num_layer=3).to(self.device)
        self.critic = Critic(in_dim=6, width=2*6, num_layer=3).to(self.device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=learning_rate)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=learning_rate)

        self.actor_scheduler = optim.lr_scheduler.ExponentialLR(self.actor_optimizer, self.gamma)
        self.critic_scheduler = optim.lr_scheduler.ExponentialLR(self.critic_optimizer, self.gamma)

    def train(self, b, batch_mode=True):
        """
        train agent from environment feedback
        """

        s, a, r, s_prime = b

        # update critic (value network)
        ## mask = 0 is usually used for terminal state
        ## in our simulation we don't have a terminal state
        mask = 1

        if batch_mode:
            full_s = torch.cat([item.unsqueeze(-1) for item in s[0]], dim=1).float().to(self.device)
            cv_s = torch.cat([item.unsqueeze(-1) for item in s[1]], dim=1).float().to(self.device)

            full_next_s = torch.cat([item.unsqueeze(-1) for item in s_prime[0]], dim=1).float().to(self.device)
            # cv_next_s = torch.cat([item.unsqueeze(-1) for item in s_prime[1]], dim=1).to(self.device)

            # a = a.unsqueeze(-1)
            r = r.unsqueeze(-1)

        else:
            full_s = torch.FloatTensor(s[0]).to(self.device)
            cv_s = torch.FloatTensor(s[1]).to(self.device)

            full_next_s = torch.FloatTensor(s_prime[0]).to(self.device)
            cv_next_s = torch.FloatTensor(s_prime[1]).to(self.device)

        pred_value = self.critic(full_s)
        targ_value = r + self.gamma * self.critic(full_next_s) * mask
        value_loss = F.mse_loss(pred_value, targ_value.detach())

        ## optimizer take step
        self.critic_optimizer.zero_grad()
        value_loss.backward()
        self.critic_optimizer.step()
        self.critic_scheduler.step()

        # update actor (policy network)
        # 64,1
        advantage = (targ_value - pred_value).detach()  # not backpropagated

        # refer to https://pytorch.org/docs/stable/distributions.html
        # 64,3
        probs = self.actor(cv_s) # cv_s
        
        if batch_mode:
            # 64, 64
            log_prob = Categorical(probs).log_prob(a+1) # add one back becase our a~[-1,0,1]
        else:
            log_prob = Categorical(probs).log_prob(torch.tensor(a+1)) # add one back becase our a~[-1,0,1]

        policy_loss = - advantage * log_prob.unsqueeze(-1)
        entropy_loss = self.entropy_weight * -log_prob.unsqueeze(-1)  # entropy maximization

        # update policy
        self.actor_optimizer.zero_grad()
        (policy_loss + entropy_loss).mean().backward()
        self.actor_optimizer.step()
        self.actor_scheduler.step()

        return (value_loss + policy_loss + entropy_loss).data
