import curbside
import utils
import envs

net_xml = "seattle.net.xml"
add_xml = "seattle.add.xml"
rou_xml = "seattle.trips.xml"

class Policy:
    def __init__(self, curb_ids, policy_name='naive'):
        self.curb_ids = curb_ids
        self.policy_name = policy_name

    def forward(self, reroute_vtype=None):
        actions = []
        for curb_id in self.curb_ids:
            if self.policy_name == 'naive':
                action = self.naive_policy(reroute_vtype[curb_id])
            if self.policy_name == 'no-action':
                action = 0

            actions.append(action)
        return actions

    def naive_policy(self, curb_reroute):
        if curb_reroute['dlv'] > curb_reroute['psg']:
            action = 1  # delivery vehicle space +1
        elif curb_reroute['dlv'] < curb_reroute['psg']:
            action = -1  # passenger vehicle space +1
        else:
            action = 0
        return action


utils.generate_route(psg_park_dm_x=2, dlv_park_dm_x=4, flow_num=200, simulate_hour=12, seed=2)
env = envs.SeattleEnv(net_xml, add_xml, rou_xml, gui=True)
# policy = Policy(env.curb_ids, policy_name='no-action')
policy = Policy(env.curb_ids, policy_name='no-action')

total_reroute = []
total_failed_parking = []

while True:
    actions = policy.forward(env.reroute_vtype)
    done = env.batch(actions, 3600)
    total_reroute.append(env.reroute_total)
    total_failed_parking.append(env.failed_parking)
    if done:
        break

env.terminate()