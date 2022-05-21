#!/usr/bin/env python
# coding: utf-8

# In[54]:


import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

get_ipython().run_line_magic('matplotlib', 'inline')


# ## Helper Functions

# In[55]:


def variable_info(variable):
    """
    Provided a variable, prints the type and content of the variable
    """
    print("This variable is a {}".format(type(variable)))
    if type(variable) == np.ndarray:
        print("\n\nThe shape is {}".format(variable.shape))
    print("\n\nThe data contained in the variable is : ")
    print(variable)
    print("\n\nThe elements that can be accessed in the variable are :\n")
    print(dir(variable))
    
variable_info(np.array([1]))


# ## Map Initialisation
# 
# Here is the function provided to create an empty plot of the size max_val with the grid lines in the desired positions. You do not need to understand the details of this function as it is purely related to the plotting.

# In[56]:


def create_empty_plot(max_val):
    """
    Helper function to create a figure of the desired dimensions & grid
    
    :param max_val: dimension of the map along the x and y dimensions
    :return: the fig and ax objects.
    """
    fig, ax = plt.subplots(figsize=(7,7))
    
    major_ticks = np.arange(0, max_val+1, 5)
    minor_ticks = np.arange(0, max_val+1, 1)
    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_ylim([-1,max_val])
    ax.set_xlim([-1,max_val])
    ax.grid(True)
    
    return fig, ax


# To create the map on which we are going to apply the path planning algorithm, we are making use of :
# - a random generator that creates a map of size max_val x max_val (defined in the previous cell)
# - a random seed to ensure that the ouptut of the random generator is the same at each iteration
# - a threshold value which determines for each cell whether it is occupied (i.e. value set to 1) or free (i.e. value set to 0). 
# 
# You can play with these parameters to change the size of the map (map_size), density of the obstacles in the map (threshold) as well as their position (random seed). 
# 
# 

# In[57]:


#Creating the grid
max_val = 50 # Size of the map

fig, ax = create_empty_plot(max_val)

# Creating the occupancy grid
np.random.seed(0) # To guarantee the same outcome on all computers
data = np.random.rand(max_val, max_val) * 20 # Create a grid of 50 x 50 random values
cmap = colors.ListedColormap(['white', 'red']) # Select the colors with which to display obstacles and free cells

# Converting the random values into occupied and free cells
limit = 12 
occupancy_grid = data.copy()
occupancy_grid[data>limit] = 1
occupancy_grid[data<=limit] = 0

# Displaying the map
ax.imshow(occupancy_grid.transpose(), cmap=cmap)
plt.title("Map : free cells in white, occupied cells in red");


# 
# If you use the default values provided to construct the map, then the outcome of the A* algorithm that you should expect to obtain is the following (with the plotting functions already pre-implemented). 
# 
# 
# <img src="Images/Astar_sol.png" alt="Drawing" style="width: 500px;"/>
# 
# - white cells : free cells
# - red cells : occupied cells
# - orange nodes : explored nodes
# - blue nodes : nodes corresponding to the optimal path
# - green circle : starting node
# - purple circle : target node
# 

# ## Implementing the Algorithm
# 
# The following implementation of the A* algorithm is based on the pseudo-code provided here : https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
# 
# Make sure that you understand the different steps of the algorithm before starting to implement it. You are free however to choose another pseudo-code that corresponds to the A* algorithm implementation if you feel more comfortable with it. 

# The following functions provide the list of possible movements depending on whether you decide to have the algorithm work in a 4 connected or an 8 connected grid ([see here for a table recapitulating the difference](https://edoras.sdsu.edu/doc/matlab/toolbox/images/morph12.html)). You do not need to edit anything in these functions.

# In[58]:


def _get_movements_4n():
    """
    Get all possible 4-connectivity movements (up, down, left right).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1)
    (up, down, left, right and the 4 diagonals).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]


# Now you can implement the A* algorithm using either the pseudo-code provided and following the structure below or selecting another one.

# In[59]:


def reconstruct_path(cameFrom, current):
    """
    Recurrently reconstructs the path from start node to the current node
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n 
                     currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path

def A_Star(start, goal, h, coords, occupancy_grid, movement_type, max_val_x, max_val_y):
    """
    A* for 2D occupancy grid. Finds a path from start to goal.
    h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param start: start node (x, y)
    :param goal_m: goal node (x, y)
    :param occupancy_grid: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    
    # -----------------------------------------
    # DO NOT EDIT THIS PORTION OF CODE
    # -----------------------------------------
    
    # Check if the start and goal are within the boundaries of the map
    '''for point in [start, goal]:
        for coord in point:
            assert coord>=0 and coord<max_val, "start or end goal not contained in the map" '''

    assert start[0]>=0 and start[0]<max_val_x and start[1]>=0 and start[1]<max_val_y, "start not contained in the map"
            
    assert goal[0]>=0 and goal[0]<max_val_x and goal[1]>=0 and goal[1]<max_val_y, "end goal not contained in the map"        
    
    # check if start and goal nodes correspond to free spaces
    if occupancy_grid[start[0], start[1]]:
        raise Exception('Start node is not traversable')

    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    
    # get the possible movements corresponding to the selected connectivity
    if movement_type == '4N':
        movements = _get_movements_4n()
    elif movement_type == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')
    
    # --------------------------------------------------------------------------------------------
    # A* Algorithm implementation - feel free to change the structure / use another pseudo-code
    # --------------------------------------------------------------------------------------------
    
    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]
    
    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]
    #fScore[start] = h(start)
    # while there are still elements to investigate
    while openSet != []:
        
        #the node in openSet having the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet
        
        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)
        
        #for each neighbor of current:
        for dx, dy, deltacost in movements:
            
            neighbor = (current[0]+dx, current[1]+dy)
            
            # if the node is not in the map, skip
            #continue skip ce tour de boucle et commence le prochain
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue
            
            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                continue
            # neighbor is in the map, not in a occupied block or already visited
            
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            # deltacost permet d aller de current vers neighbor?
            if neighbor not in openSet:
                openSet.append(neighbor)
                
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                 
    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet


# In[60]:


start = (0,0)
goal = (10,10)

'''    for point in [start, goal]:
    for p in point:
        #p = int(p)
        #print(p)
        print(p<1)
        print(int(p[0]),int(p[1]))'''
#print(start[0])


# In[61]:


# Define the start and end goal
start = (0,0)
goal = (43,33)



# -----------------------------------------
# DO NOT EDIT THIS PORTION OF CODE - 
# EXECUTION AND PLOTTING OF THE ALGORITHM
# -----------------------------------------
    
    
# List of all coordinates in the grid
x,y = np.mgrid[0:max_val:1, 0:max_val:1]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y
pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
coords = list([(int(x[0]), int(x[1])) for x in pos])

# Define the heuristic, here = distance to goal ignoring obstacles
h = np.linalg.norm(pos - goal, axis=-1)
h = dict(zip(coords, h))

# Run the A* algorithm
path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid, "4N", max_val,max_val)
path = np.array(path).reshape(-1, 2).transpose()
visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()

# Displaying the map
fig_astar, ax_astar = create_empty_plot(max_val)
ax_astar.imshow(occupancy_grid.transpose(), cmap=cmap)

# Plot the best path found and the list of visited nodes
ax_astar.scatter(visitedNodes[0], visitedNodes[1], marker="o", color = 'orange');
ax_astar.plot(path[0], path[1], marker="o", color = 'blue');
ax_astar.scatter(start[0], start[1], marker="o", color = 'green', s=200);
ax_astar.scatter(goal[0], goal[1], marker="o", color = 'purple', s=200);


# <blockquote>
# 
# <span style="color: #2980B9 ;">
#     
# Note that the heursitic here was chosen to be the Euclidian distance between the starting point and the goal. The choice of heuristic guides the search and will determine the speed at which the local optimum is found. A bad heursitic, such as a Manhattan distance in an 8 connected grid will however prevent you from finding the optimal solution because it does not take diagonals into account. 
# 
# 
# </blockquote>

# In[ ]:




