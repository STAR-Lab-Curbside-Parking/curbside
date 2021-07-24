import torch
import torch.nn as nn
from torch import optim

# A2C
class AC(nn.Module):
    def __init__(self, input_dim=7, hidden_dim=16, num_actions=3):
        super(AC, self).__init__()

        self.fc = nn.Sequential(nn.Linear(input_dim, hidden_dim), 
                                nn.ReLU())
        self.pi = nn.Linear(hidden_dim, num_actions)
        self.v = nn.Linear(hidden_dim, 1)

        self.gamma = 0.9
        self.optimizer = optim.Adam(self.parameters(), lr=0.001)

    def forward(self, state):

        x = self.fc(torch.from_numpy(state).float())

        pi_out = self.pi(x)
        v_out = self.v(x)

        return pi_out, v_out

    def compute_target(self, v, r):
        """
        compute TD target
        """
        G = r + self.gamma * v

        return G
