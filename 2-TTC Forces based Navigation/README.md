# ASSIGNMENT 2: TTC Forces based Local Navigation
## Submitted by: Huzefa Shabbir Hussain Kagalwala

This repository contains two folders:
1. The **Basic** folder contains code for the vanilla implementation of the TTC forces model.
2. The **Extra Credits** folder contains further subfolders on the PowerLaw Model implementation instead of TTC; Adverserial Uncertainty and Velocity Perturbation methods to update the forces on the agent using the TTC model.

### Basic Implementation
These are the following parameters for the basic implementation:
ksi = 0.5 (The relaxation time to compute the goal force)
Sensing Horizon (dhor) = 10
Time Horizon (timehor) = 5
Maximum Force = 10
The maximum speed, goal speed, radius and other parameters are extracted from the CSV
Note: The initial 4 values mentioned here are hard-coded in the script `agent.py`.

### EXTRA CREDITS:

#### POWER LAW:
The parameters mentioned in the **BASIC REQUIREMENTS** are also common for this scenario
tau_0 = 3 (Exponential cut-off point)
These are the seperate parameters for the epsilion = 0 and 0.02:
1. EPSILON = 0.2 (Velocity uncertainty constant)
   - k = 1.05 (PowerLaw scaling factor)
   - m = 1 (PowerLaw constant)
2. EPSILON = 0 (Velocity uncertainty constant
   - k = 1.272 (PowerLaw scaling factor)
   - m = 1 (PowerLaw constant)

#### VELOCITY PERTURBATION:
1. Radius of disk from which velocity perturbations are sampled (nu) = 0.1
2. epsilon = 0.2 (Velocity uncertainty constant)
3. Other parameters are common. The TTC model is used here, instead of PowerLaw

#### ADVERSARIAL UNCERTAINTY:
1. Same parameters as Velocity Perturbation
2. TTC Model is used here instead of of PowerLaw

### Usage Instructions:
1. You will require NumPy, ScyPy, Matplotlib, Tkinter to run this code.
2. `agent.py` is the file which contains the code to perform sampling-based navigation for disc shaped agents.
3. To run the visualization, run the `simulator.py` file.
4. To change the number of agents, change the file name in Line 21 of `simulator.py` to either **3_agents.csv** or **8_agents.csv**.
5. Upon running the `simulator.py` file, you will generate a .csv file which contain the ID, location and velocities of all the agents at each time step of the simulation.
