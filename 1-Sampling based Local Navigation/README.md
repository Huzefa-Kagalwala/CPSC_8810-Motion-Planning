### CPSC 8810 - Motion Planning
#### ASSIGNMENT 1: Sampling-Based Local Navigation
#### Submitted by: Huzefa Shabbir Hussain Kagalwala
#### Submission date: 01-26-2020

#### Variable Description:
1. N = The sample size over which random velocities are generated
2. cand_vel = Creating a matrix to store candidate velocities' x and y components
3. neighbor_pos = Array which stores the positions of an agent which is considered a "neighbor"
4. neighbor_size = Array which stores the radius of an agent which is considered a "neighbor"
5. neighbor_vel = Array which stores the velocity of an agent which is considered a "neighbor"
6. neighbor_id = Array which stores the velocity of an agent which is considered a "neighbor"
7. alpha = The weightage of how much the cost function prioritizes the relative velocity between the goal and candidate
8. beta = The weightage of how much the cost function prioritizes the relative velocity between the self velocity and candidate
9. gamma = The weightage of how much the cost function prioritizes the safety (time to collision)
10. vel_array = Array containing the uniform random disributions of the velocities
11. theta = Array containing the uniform random disributions of the angles
12. sample_vel_x and sample_vel_y = The arrays containing the x,y coordinates of the candidate velocities
13. cand_vel = The matrix with the candidate velocities in vector form
14. goal_dist = This is the distance between the agent and its goal position
15. cost1, cost2 and cost3 = The arrays containing the 3 terms to compute the final cost function
16. fin_vel = The final computed velocity which will be updated

#### Usage Instructions:
1. You will require NumPy, ScyPy, Matplotlib, Tkinter to run this code.
2. `agent.py` is the file which contains the code to perform sampling-based navigation for disc shaped agents.
3. To run the visualization, run the `simulator.py` file.
4. To change the number of agents, change the file name in Line 21 of `simulator.py` to either **3_agents.csv** or **8_agents.csv**.
5. Upon running the `simulator.py` file, you will generate two .csv files which contain the ID, location and velocities of all the agents at each time step of the simulation.
