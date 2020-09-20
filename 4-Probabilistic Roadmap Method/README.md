# ASSIGNMENT 3: Probablistic Roadmap Method (Sampling based Navigation)
## Submitted by: Huzefa Shabbir Hussain Kagalwala

## Code Walk-through:
### CONSTRUCTION PHASE:
1. **Sampling:**
   First we sample points by using meshgrid and adding some noise to the samples to induce randomness. The randomness has been added by    taking the width of the samples and based on that, we generate random samples. These samples are then added to the meshgrid points. The    "poinGenerator()" function takes the meshgrid limits, the spacing required and the "noise factor" we want. A factor of 1, means no noise    (due to data type mismatch, to have no noise, use 0.999999, instead of 1) and anything other than that will add noise.
2. **Adding the vertices:**
   Next, the vertices were added inside the configuration space by inflating all the obstacles with their robot radius. After adding all    the points inside the obstacle-free configuration space, edges are added between the vertices that are within a certain distance from a    certain vertex. 
3. **Adding edges:**
   To avoid vertices from connecting through obstacles to form edges, an "interpolate" function was created to check the local path between    two given configurations. It divides the edge into "n" number of parts and checks these points if they're in obstacles. After making    these checks, another check is performed to connect configurations if they are not previously connected by using the    connectedComponentNr variable. If it is -1 then the vertices aren't connected. If the vertices are already connected, then separate    loops were run to trace back and find the parents of the particular configuration. In order to achieve this, first the previous parents    are stored with the help of connectedComponentNr and then the vertices of every edge from the set of parents are computed and returned    as a list.

### Query Phase:
1. Here, the A* search algorithm is used to compute the path . This was done through "find_path" function which takes in start node, goal    node and the graph as arguments. 
2. **Adding the query nodes:**
   *q_s* and *q_g* are variables to add the start and goal vertex inside the obstacle-free C-space. To add the edge from the start to its      nearby vertices, the same process during the construction phase was repeated to add edges. 
3. **A***:
   Parent variable which is an empty tuple is used to backtrack to get the path. Euclidean distance from the current node to goal node is    taken as the heuristic. A* algorithm is run by popping the start node from the open set and adding to the closed set. The path to take    from the start node to the next node is computed on the basis of heuristic and g_value. If the neighboring node is not in closed set and    not in open set, then the neighboring node is pushed based on the f-value. If during random query, any of the start or goal nodes lie in    the obstacle space and you try to generate a path, the code enters into an infinite loop and does not return anything which means it is    unable to find a path to the goal node.  

## Instructions
To run this code, run the `prmplanner.py` file
