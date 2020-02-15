import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib import colors
import random
import copy
import math
import heapq
import time

# Global variable of the 2D array of the map
#MAP = []

def main():

    #global MAP
    MAP = []
    dim = 0
    p = 0

    try:
        dim = int(input("Please enter the DIM: "))
        p = float(input("Please enter the P-Value: "))
    except ValueError:
        print("Invalid Dim/P-Value")
        exit()


    #MAP = create_map(dim, p)
    printMap(MAP)


"""
Prints the Map
"""
def printMap(map):

    cmap = colors.ListedColormap(['white', 'black', 'green'])
    bounds = [0,1,2]

    #plt.imshow(map, cmap=cmap, vmin=0,vmax=2)

    benchAX = plt.axes([0.001, 0.5, 0.15, 0.05])
    benchBtn = Button(benchAX, 'Benchmark', color='red', hovercolor='green')
    benchBtn.on_clicked(benchmark)


    plt.show()


"""
Function to generate and return a map (2D Array) given DIM and P
"""
def create_map(dim, p):

    while True:

        map = [[0 for n in range(dim)] for n in range(dim)]

        for i in range(dim):
            for j in range(dim):
                #randomNum = random.randrange(0,2,1)
                prob = np.random.choice(np.arange(0,2), p=[(1-p), p])
                if(prob == 1):
                    if((i == 0 and j == 0) or (i == dim-1 and j == dim-1)):
                        continue
                    map[j][i] = 1


        # Check is to make sure it can get from start to goal
        check = False
        dim = len(map[0])
        check = dfs_algo(dim-1,dim-1,map)

        # Generate map that can be solved
        if (check == False):
            continue
        else:
            break

    return map

"""
Algorithm for DFS. Used to see if it is possible to reach given point from (0,0) in a 2D map
"""
def dfs_algo(dy,dx,map):

    total_discovered = 0

    success = False

    stack = []
    discovered = []
    previous = []

    # Push starting node onto stack
    stack.append([0,0])
    discovered.append([0,0])


    while len(stack):
        
        cur = stack.pop()
        y = cur[0]
        x = cur[1]

        # Found end cell
        if(x == dx and y == dy):
            success = True
            break
        
        # Found wall, ignore and continue
        if(map[y][x] == 1):
            continue

        for i in range(4):
            if i is 3: # down
                if ([y+1,x] in discovered) or (y+1 > dy):
                    continue
                previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                stack.append([y+1,x])
                discovered.append([y+1,x])
                total_discovered += 1
            elif i is 2: # right
                if ([y,x+1] in discovered) or (x+1 > dx):
                    continue
                previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
                stack.append([y,x+1])
                discovered.append([y,x+1])
                total_discovered += 1
            elif i is 1: # up
                if ([y-1,x] in discovered) or (y-1 < 0):
                    continue
                previous.append({'cur' : [y-1,x], 'prev' : [y,x]})
                stack.append([y-1,x])
                discovered.append([y-1,x])
                total_discovered += 1
            else: # left
                if ([y,x-1] in discovered) or (x-1 < 0):
                    continue
                previous.append({'cur' : [y,x-1], 'prev' : [y,x]})
                stack.append([y,x-1])
                discovered.append([y,x-1])
                total_discovered += 1

    # DFS done
    if(success):
        return True
    else:
        return False

"""
Distance functions for A Star heuristic estimates
x1 y1 is current cell
x2 y2 is goal
"""
def euclidean_distance(x1,y1,x2,y2):
    return math.sqrt(((x1-x2)**2) + ((y1-y2)**2))

def manhattan_distance(x1,y1,x2,y2):
    return (abs(x1-x2) + abs(y1-y2))



"""
Algorithm for A-Star using Euclidean distance
"""
def astar_euc(map):

    #map = copy.deepcopy(MAP)

    dim = len(map[0])
    total_discovered = 0

    success = False

    start = time.process_time()

    # Priority queue
    open = []
    previous = []
    heapq.heapify(open)

    closed = []

    preH = euclidean_distance(0,0,dim-1,dim-1)
   
    # ( f value, h value, g value, coordinate on map, parent)
    heapq.heappush(open, (preH, preH, 0, [0,0], [0,0]))


    while len(open):

        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curG = cur[2]
        coord = cur[3]
        y = coord[0]
        x = coord[1]

        previous.append({'cur' : cur[3], 'prev' : cur[4]})

        """
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """

        # Found the end
        if(x == dim-1 and y == dim-1):
            heapq.heappush(open, cur)
            success = True
            break

        # Found wall, ignore and continue
        if(map[y][x] == 1):
            continue

        closed.append(coord)

        # generate cells around
        # for each neighbor, check if in closed list or if already exist in open list.
        for i in range(4):
            if i is 0: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (map[y+1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x,y+1,dim-1,dim-1)
                fScore = gScore + hScore

                #previous.append({'cur' : [y+1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y+1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y+1,x], [y,x]))
                    total_discovered += 1
            elif i is 1: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (map[y][x+1] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x+1,y,dim-1,dim-1)
                fScore = gScore + hScore

                #previous.append({'cur' : [y,x+1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x+1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y,x+1], [y,x]))
                    total_discovered += 1
            elif i is 2: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (map[y-1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x,y-1,dim-1,dim-1)
                fScore = gScore + hScore

                #previous.append({'cur' : [y-1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y-1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y-1,x], [y,x]))
                    total_discovered += 1
            else: # left

                # Already in closed list, skip it
                if ([y,x-1] in closed) or (x-1 < 0) or (map[y][x-1] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x-1,y,dim-1,dim-1)
                fScore = gScore + hScore
    
                #previous.append({'cur' : [y,x-1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x-1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y,x-1], [y,x]))
                    total_discovered += 1
    
    end = time.process_time()
    total_time = end - start
  
    # A-Star done, traceback the 'previous' list to generate path to display

    if(success):
        return total_time
    else:
        return -1


def benchmark(e):


    list = []

    # For each DIM starting at 10 and going up by 10, test it against a P-Value on a newly generated map 50 times
    dimTest = 10
    for dv in range(10):
        pTest = 0
        for pv in range(5):
            for i in range(1):
                m = create_map(dimTest,pTest)
                time_taken = astar_euc(m)

                list.append([dimTest, {'DIM' : dimTest, 'Time' : time_taken, 'P' : pTest}])

            pTest += 0.1
        dimTest += 10
    print(list)


if __name__ == '__main__':
    main()