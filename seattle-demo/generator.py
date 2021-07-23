import torch
import torch.nn as nn
import torch.nn.functional as F 
from torch import optim

# create simple forward network

class simple_nn(nn.Module):
    def __init__(self, hidden_state=24):
        super(simple_nn, self).__init__()
        # self.start_matrix = torch.empty(12, 4, requires_grad=True)
        # nn.init.kaiming_normal_(self.start_matrix)
        
        self.fc1 = nn.Linear(7, hidden_state)
        self.fc2 = nn.Linear(hidden_state, 3)
    
    def forward(self, state):
        action_space = [-1, 0, 1]
        # out = state * self.start_matrix
        # out = F.relu(torch.sum(out, dim=1))
        out = F.relu(self.fc1(state))
        out = F.relu(self.fc2(out))
        out = torch.argmax(F.softmax(out, dim=-1), dim=0).detach().numpy()
        action = action_space[out]
        return action


# linear agent
class linear_agent:
    def __init__(self):
        self.nn = simple_nn(hidden_state=24)
        self.optimizer = optim.Adam(self.nn.parameters(), lr=0.001)
        
    def forward(self, state):
        return self.nn(state)
    
    def optimize(self, reward):
        self.optimizer.zero_grad()
        loss = torch.tensor(reward, dtype=torch.float, requires_grad=True)
        loss.backward()
        self.optimizer.step()

class Policy:
    def __init__(self, curb_ids, policy_name='no-action'):
        self.curb_ids = curb_ids
        self.policy_name = policy_name
        if self.policy_name == 'nn':
            # only one agent
            self.agent = linear_agent()
            
    def forward(self, states):
        actions = []
        for state, curb_id in zip(states, self.curb_ids):
            # no action
            if self.policy_name == 'no-action':
                action = 0
            # last step experience
            if self.policy_name == 'last_min':
                action = self.last_min_policy(state)
            
            if self.policy_name == 'nn':
                action = self.nn_policy(state, curb_id)
            
            actions.append(action)
        
        return actions
    
    def last_min_policy(self, state):
        if state[0,0] > state[1,0]:
            action = 1 # delivery vehicle space +1
        elif state[0,0] < state[1,0]:
            action = -1 # delivery vehicle space -1
        else:
            action = 0
        return action
    
    def nn_policy(self, state, curb_id):
        # only one agent
        action = self.agent.forward(torch.tensor(state, dtype=torch.float))
        
        return action