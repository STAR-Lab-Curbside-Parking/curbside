import torch
import torch.nn as nn
from torch import optim
import torch.nn.functional as F
from torch.autograd import Variable

# A2C
class A2C(nn.Module):
    def __init__(self, input_dim=7, hidden_dim=16, num_actions=3):
        super(A2C, self).__init__()

        self.fc = nn.Sequential(nn.Linear(input_dim, hidden_dim), 
                                nn.ReLU())
        self.pi = nn.Linear(hidden_dim, num_actions)
        self.v = nn.Sequential(nn.Linear(hidden_dim, hidden_dim), 
                               nn.ReLU(),
                               nn.Linear(hidden_dim, 1))

        self.gamma = 0.9
        self.optimizer = optim.Adam(self.parameters(), lr=0.001)

        self.policy = None
        self.value = None

    def forward(self, state):

        x = self.fc(torch.from_numpy(state).float())

        pi_out = self.pi(x)
        v_out = self.v(x)

        return pi_out, v_out

    def train(self, s, a, r, s_prime):
        """
        train agent from environment feedback
        """
        self.policy, self.value = self.forward(s)
        _, v_prime = self.forward(s_prime)

        td_target = r + self.gamma * v_prime.data
        advantage = td_target - self.value.data

        probs = F.softmax(self.policy, dim=0)
        log_probs = F.log_softmax(self.policy, dim=0)
        log_action_probs = log_probs.gather(0, Variable(a + 1))

        policy_loss = (-log_action_probs * advantage).sum()
        value_loss = (.5 * advantage ** 2).sum() # (self.value - r)
        entropy_loss = (log_probs * probs).sum()

        loss = policy_loss + value_loss + entropy_loss

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        return
