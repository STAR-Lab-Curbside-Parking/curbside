import argparse
import numpy as np
import random, timeit
import torch

import agent, memory, utils, simulation

torch.manual_seed(0)
np.random.seed(0)
random.seed(0)

NET_XML = "ring.net.xml"
ADD_XML = "ring.add.xml"
ROU_XML = "ring.trips.xml"

parser = argparse.ArgumentParser()
parser.add_argument("-f", "--flow", help="parking flow ratio cv to ncv", type=float)
parser.add_argument("-d", "--duration", help="parking duration ratio cv to ncv", type=float, default=0.33)
parser.add_argument("-c", "--cap", help="cv cap at start", type=int, choices=range(1,10))
parser.add_argument("-s", "--seed", help="random seed for route generation", type=int)
args = parser.parse_args()

def main():

    # 1, 0.33
    utils.generate_route(length=3600, park2curb=1, background2park=0, 
                         cv2ncv_pf=args.flow, cv2ncv_pd=args.duration, 
                         seed=args.seed)

    Model = agent.A2C(
        batch_size=64,
        learning_rate=0.001,
        gamma=0.95
    )

    Memory = memory.Memory(
        10000,
        50
    )

    Simulation = simulation.Simulation(
        Model, 
        Memory,
        training_epochs=3,
        cv_cap=args.cap
    )

    total_episodes = 1
    episode = 0
    timestamp_start = timeit.default_timer()
    
    while episode < total_episodes:

        print('\n----- Episode', str(episode+1), 'of', str())
        # epsilon = 1.0 - (episode / config['total_episodes'])
        simulation_time, training_time, res = Simulation.run(episode) # epsilon
        print('Simulation time:', simulation_time, 's - Training time:', training_time, 's - Total:', round(simulation_time+training_time, 1), 's')
        episode += 1

    print("\n----- Start time:", timestamp_start)
    print("----- End time:", timeit.default_timer())

    # print(Memory._size_now())
    # print("----- Session info saved at:", path)
    with open("mp_baseline.txt", "a") as myfile:
        myfile.write("{},{},{},{},{}\n".format(
            args.flow,
            args.duration,
            args.cap, 
            args.seed,
            res
        ))

if __name__ == "__main__":
    main()