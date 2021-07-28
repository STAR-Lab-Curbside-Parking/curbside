# curbside

This repo contains two simulation environments for urban curbside parking management
+ seattle-demo: configuration is drawn from downtown Seattle; basic parking demand is extracted from transaction data;
+ ring: hypothetical for faster algorithm testing.

Simulation is built on [SUMO](https://sumo.dlr.de/docs/index.html), because it is easier to define and perform microscopic physical vehicle maneuvers here, compared to in other simulation environments. A heavily used SUMO functionality is [TraCI](https://sumo.dlr.de/docs/TraCI.html), which serves as the Python interface for our microscopic control.

The goal of this project is to dynamically manage curb resources to better accomodate on-street parking demand, by various types of vehicles, e.g., **cv vs non-cv**, and **delivery vs passenger**. This demand challenges current curbside management system because the parking durations can be drastically different (think of ridesharing vehicles picking up passengers vs private vehicles parking on-street) Conflicts, in both spatial and temporal domain, exist between maximizing individual utilities against system-level optimum.

During the discussions since Jan 2021, we think there are three parties in this game: management center, vehicles, and the curbs (infrastructure). There are essentially 2 basic angles to look at this problem
+ **solve for an optimal matching decision**, which the center makes, that considers spatiotemporal features of the game and matches the two sides of the curbside parking market optimally.
+ **control infrastructure**, i.e., curbside spaces, to adjust the spaces within each of them to accomodate the vehicular parking demand. For example, at `time 0`, 5 spaces are for CV and 3 are for non-CV; then at `time 1`, the space allocation can be adjusted to 4 spaces for CV and 4 for non-CV. The infrastructure performed this action (`-1` for CV space) probably because it seems more non-CVs are not served.

This repo is performing the second - controlling the curbs space allocations - to answer the curbside parking problem.

## seattle-demo

**Seattle-demo** environment is our first developed simulation environment. It contains 4 groups of documents
+ CSV: real-world data that we leveraged for buildling up the environment.
+ XMLs: required to define SUMO simulation
    - `*.sumocfg`: SUMO simulation configuration file
    - `*.net.xml`: SUMO network topology definition, e.g., geometries and objects.
    - `*.add.xml`: SUMO additional object file, where **parking area**s are defined. Check [here](https://sumo.dlr.de/docs/sumo.html#format_of_additional_files) for how it is formatted.
    - `*.backup.xml`: our manual backup of past versions. 
+ Jupyter notebook
    - `*_*.ipynb`: numbered from **01** to **11**, these Notebooks are chronically ordered during our development. We started from some simple functions to set up the simulation.
+ Python files
    - `utils.py`, `envs.py`, `curbside.py`, `generator.py`, `main.py`: consolidated Python code to simulate and train models.

During our exploration, we found that there are many uncertainties in the **Seattl-demo** simulation: parking demand, vehicle types (cv vs non-cv, delivery vs passenger), how they cruise in the network, roadway geometries (one-way streets), state and reward definition, etc. Learning on these settings could be difficult.

## ring

Answering these questions, we built a simple set-up that is more configurable and faster for algorithm testing. And this is **ring** subfolder.

This environment has the latest efforts. Its file system is similar to those in the **seattle-demo**.
+ XMLs: define the SUMO simulation
+ Python:
    - `main.py`: main iteration
    - `utils.py`: functions that prepare for simulation, e.g., route generation.
    - `envs.py`: defines the physical world - how vehicles move, request parking, are rejected and rerouted.
        * `curbside.py`: class of **smart curb**s, that we are controlling.
    - `agent.py`: could be a place where RL agnets are defined
```bash
main.py
|-- envs.py
|   |
|   |-- curbside.py
|
|-- agent.py
|
|-- utils.py
```