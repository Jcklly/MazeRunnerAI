import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib import colors
import random
import copy
import math
import heapq
import time

"""
Global Vars
"""
MAP = []
dim = 50
p = 0.2
            
"""
Creates new map
"""
def create_new_map(e):
    
    # map = copy.deepcopy(MAP)
    dim = len(MAP[0])

    for i in range(dim):
        for j in range(dim):
            MAP[j][i] = 0
            #randomNum = random.randrange(0,2,1)
            prob = np.random.choice(np.arange(0,2), p=[(1-p), p])
            if(prob == 1):
                if((i == 0 and j == 0) or (i == dim-1 and j == dim-1)):
                    continue
                MAP[j][i] = 1
    print("created new map")
    plt.clf()
    printMap(MAP)

"""
Prints the Map
"""
def printMap(MAP):

    cmap = colors.ListedColormap(['white', 'black', 'green'])
    bounds = [0,1,2]

    plt.imshow(MAP, cmap=cmap, vmin=0,vmax=2)

    bfsAX = plt.axes([0.001, 0.7, 0.1, 0.05])
    bfsBtn = Button(bfsAX, 'BFS', color='red', hovercolor='green')
    bfsBtn.on_clicked(bfs_algo)

    mapAX = plt.axes([0.001, 0.3, 0.15, 0.05])
    mapBtn = Button(mapAX, 'New map', color='red', hovercolor='green')
    mapBtn.on_clicked(create_new_map)

    plt.show()

"""
Algorithm for BFS
"""
def bfs_algo(e):

    map = copy.deepcopy(MAP)
    dim = len(MAP[0])
    total_discovered = 0

    success = False

    # Start timer for algorithm
    start = time.process_time()

    queue = []
    discovered = []
    previous = []
    
    queue.append([0,0])
    discovered.append([0,0])

    while queue:
        # Get all valid cells around current spot and add to queue, including previous node for each cell to be able to traceback path
        cur = queue.pop(0)       
        y = cur[0]
        x = cur[1]

        # Found end cell
        if(x == dim-1 and y == dim-1):
            success = True
            break
        
        # Found wall, ignore and continue
        if(MAP[y][x] == 1):
            continue

        # Look at adjacent cells
        for i in range(4):
            if i is 0: # down
                if ([y+1,x] in discovered) or (y+1 > dim-1):
                    continue
                discovered.append([y+1,x]) 
                queue.append([y+1,x])
                previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                total_discovered += 1
            elif i is 1: # right
                if ([y,x+1] in discovered) or (x+1 > dim-1):
                    continue
                discovered.append([y,x+1])
                queue.append([y,x+1])
                previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
                total_discovered += 1
            elif i is 2: # up
                if ([y-1,x] in discovered) or (y-1 < 0):
                    continue
                discovered.append([y-1,x])
                queue.append([y-1,x])
                previous.append({'cur' : [y-1,x], 'prev' : [y,x]})
                total_discovered += 1
            else: # left
                if ([y,x-1] in discovered) or (x-1 < 0):
                    continue
                discovered.append([y,x-1])
                queue.append([y,x-1])
                previous.append({'cur' : [y,x-1], 'prev' : [y,x]})
                total_discovered += 1

    end = time.process_time()
    total_time = end - start

    # BFS done, traceback the 'previous' list to generate path to display

    # Check if algorithm failed and did not find a path
    if(not success):
        print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: BFS\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
        return

    a,b = -1,-1
    count = 0

    for i,d in enumerate(previous):
        if d['cur'] == [dim-1,dim-1]:
            a,b = previous[i].values()
            break

    while True:
        
        # Show the last spot it got to before giving up
        if a is -1 or b is -1:
            a,b = previous[-1].values()

        prevX = b[1]
        prevY = b[0]

        if (prevY is 0) and (prevX is 0):
            break

        map[prevY][prevX] = 2
        count += 1

        for i, d in enumerate(previous):
            if d['cur'] == [prevY,prevX]:
                a,b = previous[i].values()
                break

    
    print('**********************\n' + 'Algorithm: BFS\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')


    # Refresh the map 
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    plt.clf()
    printMap(map)

"""
Main code
"""
MAP = [[0 for n in range(dim)] for n in range(dim)]
for i in range(dim):
    for j in range(dim):
        #randomNum = random.randrange(0,2,1)
        prob = np.random.choice(np.arange(0,2), p=[(1-p), p])
        if(prob == 1):
            if((i == 0 and j == 0) or (i == dim-1 and j == dim-1)):
                continue
            MAP[j][i] = 1
printMap(MAP)