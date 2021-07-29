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

def main():

    utils.generate_route(length=3600, park2curb=1, background2park=0, cv2ncv_pf=2, cv2ncv_pd=0.33)

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
        training_epochs=3
    )

    total_episodes = 40
    episode = 0
    timestamp_start = timeit.default_timer()
    
    while episode < total_episodes:

        print('\n----- Episode', str(episode+1), 'of', str())
        # epsilon = 1.0 - (episode / config['total_episodes'])
        simulation_time, training_time = Simulation.run(episode) # epsilon
        print('Simulation time:', simulation_time, 's - Training time:', training_time, 's - Total:', round(simulation_time+training_time, 1), 's')
        episode += 1

    print("\n----- Start time:", timestamp_start)
    print("----- End time:", timeit.default_timer())

    # print(Memory._size_now())
    # print("----- Session info saved at:", path)

if __name__ == "__main__":
    main()