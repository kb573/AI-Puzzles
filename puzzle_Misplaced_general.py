import numpy as np
from copy import deepcopy
import time

def misplaced_tiles(initial,goal):
    """
    Calculate the number of misplaced tiles from the initial state compared to the goal state. 

    Inputs: initial (the initial state), goal (the goal state)
    Return: cost (the number of misplaced tiles)
    """
    cost = np.sum(initial != goal) - 1
    if cost > 0:
        return cost


def shortest_path(paths):
    """
    Takes all current states and finds the shortest path within them. 

    Inputs: state (all possible paths)
    Return: shortest (the shortest possible path to the goal state)
    """
    shortest = np.array([], int).reshape(-1, 9)
    n = len(paths) - 1
    while n != -1:
        shortest = np.insert(shortest, 0, paths[n]['initial'], 0)
        n = (paths[n]['parent'])
    return shortest


def positions(initial):
    """
    Shows the position of tiles for a given state. 

    Inputs: initial (the initial puzzle state)
    Return: position (the position of tiles for the given state)
    """
    position = np.array(range(9))
    for a, b in enumerate(initial):
        position[b] = a
    return position



def search(initial, goal):
    """
    Shows the position of tiles for a given state. 

    Inputs: initial (the initial puzzle state), goal (the goal state)
    Return: paths (all possible pathings), lens(priority_queue) (the length of the priority queue) 
    """
    transformations = np.array([('up', [0, 1, 2], -3),('down', [6, 7, 8],  3),('left', [0, 3, 6], -1),('right', [2, 5, 8],  1)], dtype =  [('move',  str, 1),('position', list),('head', int)])
    dtstate = [('initial',  list),('parent', int),('gn',  int),('hn',  int)]
    
     # initializing the cost, gn, hn, parent and paths.
    costg = positions(goal)
    gn = 0
    hn = misplaced_tiles(positions(initial), costg)
    parent = -1
    paths = np.array([(initial, parent, gn, hn)], dtstate)
    # setup of a priority queue.
    priority = [('position', int),('fn', int)]
    priority_queue = np.array( [(0, hn)], priority)

    while 1:
        # the priority queue is sorted and its 1st element is picked to evaluate and then removed from the queue.
        priority_queue = np.sort(priority_queue, kind='mergesort', order=['fn', 'position'])     
        position, fn = priority_queue[0]                                                 
        priority_queue = np.delete(priority_queue, 0, 0)                   
        initial, parent, gn, hn = paths[position]
        initial = np.array(initial)
        # use the blank square in the selected element as a starting point. 
        blank_tile = int(np.where(initial == 0)[0]) 
        start_time = time.time()
        gn += 1                              
        for x in transformations:
            if blank_tile not in x['position']:
                # deep copy the current state.
                openstates = deepcopy(initial)                   
                openstates[blank_tile], openstates[blank_tile + x['head']] = openstates[blank_tile + x['head']], openstates[blank_tile]             
                # check if a node has been previously explored.
                if ~(np.all(list(paths['initial']) == openstates, 1)).any():    
                    end_time = time.time()
                    if (( end_time - start_time ) > 2):
                        print("There was an error solving this puzzle. \n")
                        exit 
                    # get the misplaced tiles estimate.
                    hn = misplaced_tiles(positions(openstates), costg)    
                     # a new state is added to the paths list.                    
                    new_state = np.array([(openstates, position, gn, hn)], dtstate)         
                    paths = np.append(paths, new_state, 0)
                    # the f score is calculated by adding the path cost and the manhatten estimate.
                    fn = gn + hn                                        
                    new_state = np.array([(len(paths) - 1, fn)], priority)    
                    priority_queue = np.append(priority_queue, new_state, 0)
                      # Confirm the the goal state has been reached  
                    if np.array_equal(openstates, goal):                              
                        print("The puzzle has been solved. \n")
                        return paths, len(priority_queue)                     
    return paths, len(priority_queue)


 # User input to create initial state.
initial = []
print("Please provide the values for the initial state.")
print("Input the value of each tile working from left to right and downwards.")
for i in range(0,9):
    x = int(input("enter tile " + str(i + 1) + " :"))
    initial.append(x)

 # User input to create goal state.       
goal = []
print("Please provide the values for the goal state.")
print("Input the value of each tile working from left to right and downwards.")
for i in range(0,9):
    x = int(input("enter tile " + str(i + 1) + " :"))
    goal.append(x)

paths, visited = search(initial, goal) 
shortestpath = shortest_path(paths)
transformations = len(shortestpath) - 1
print('Total transformations:',transformations)
print('Total paths generated:', len(paths))