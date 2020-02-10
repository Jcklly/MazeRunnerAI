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
MAP = []
FIREPROB = 0
FIRESTART = [0,0]

def main():

    global MAP
    global FIREPROB
    dim = 0
    p = 0
    fp = 0

    try:
        dim = int(input("Please enter the DIM: "))
        p = float(input("Please enter the P-Value: "))
        fp = str(input("Please enter the Q-Value (Type R for random): "))
    except ValueError:
        print("Invalid Dim/P-Value")
        exit()

    # make fire probability random between 0 and 1
    if (fp.lower() == "r") or (fp.lower() == "random"):
        choice = [0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1]
        fp = float(random.choice(choice))
    else:
        try:
            float(fp)
        except ValueError:
            print("Invalid Q-Value")
            exit()

    FIREPROB = float(fp)
    MAP = create_map(dim, p)
    printMap(MAP)


"""
Prints the Map
"""
def printMap(map):

    cmap = colors.ListedColormap(['white', 'black', 'green', 'red'])
    bounds = [0,1,2,3]

    plt.imshow(map, cmap=cmap, vmin=0,vmax=3)

    strat1 = plt.axes([0.001, 0.5, 0.15, 0.05])
    strat1Btn = Button(strat1, 'Strategy 1', color='red', hovercolor='green')
    strat1Btn.on_clicked(astar_man)

    astar2AX = plt.axes([0.001, 0.4, 0.15, 0.05])
    astar2Btn = Button(astar2AX, 'A* Manhattan', color='red', hovercolor='green')
    astar2Btn.on_clicked(astar_man)

    plt.show()


"""
Function to generate and return a map (2D Array) given DIM and P.
Must be able to reach fire spot from beginning and be able to be completed.
"""
def create_map(dim, p):

    global FIRESTART
    fy = 0
    fx = 0

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

        # Pick random spot on map for fire to start
        fy = random.randrange(0,dim)
        fx = random.randrange(0,dim)
        
        map[0][dim-1] = 3

        # Check1 is to make sure it can get from start to goal
        # Check2 is to make sure it can get from start to fire
        check1 = False
        check2 = False
        dim = len(map[0])
        check1 = dfs_algo(dim-1,dim-1, map)
        check2 = dfs_algo(0,dim-1,map)

        # Generate map that can be solved
        if ((check1 == False) or (check2 == False)):
            continue
        else:
            break

    FIRESTART = [fy,fx]
    return map




"""
Algorithm for DFS. Used to see if it is possible to reach given point from (0,0) in a 2D map
"""
def dfs_algo(dy,dx,map):

    total_discovered = 0

    success = False

    # Start the timer
    start = time.process_time()

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
Algorithm for A-Star using Manhattan distance
"""
def astar_man(e):

    map = copy.deepcopy(MAP)

    dim = len(MAP[0])
    total_discovered = 0

    success = False

    start = time.process_time()

    # Priority queue
    open = []
    previous = []
    heapq.heapify(open)

    closed = []

    preH = manhattan_distance(0,0,dim-1,dim-1)
   
    # ( f value, h value, g value, coordinate on map, parent)
    heapq.heappush(open, (preH, preH, 0, [0,0], [0,0]))

    #print(map)

    while len(open):

        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curG = cur[2]
        coord = cur[3]
        y = coord[0]
        x = coord[1]

        previous.append({'cur' : cur[3], 'prev' : cur[4]})

        # fire spreads
        map = fire_spread(map)

        # Died in fire during this step
        if(map[y][x] == 3):
            #print("DIED IN FIRE")
            pass

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
            if i is 1: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (MAP[y+1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x,y+1,dim-1,dim-1)
                fScore = gScore + hScore

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
                    #previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                else:
                    pass
            elif i is 2: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (MAP[y][x+1] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x+1,y,dim-1,dim-1)
                fScore = gScore + hScore

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
                    #previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
            elif i is 3: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (MAP[y-1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x,y-1,dim-1,dim-1)
                fScore = gScore + hScore

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
                    #previous.append({'cur' : [y-1,x], 'prev' : [y,x]})
            else: # left

                # Already in closed list, skip it
                if ([y,x-1] in closed) or (x-1 < 0) or (MAP[y][x-1] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x-1,y,dim-1,dim-1)
                fScore = gScore + hScore
    
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
                    #previous.append({'cur' : [y,x-1], 'prev' : [y,x]})
  
    end = time.process_time()
    total_time = end - start
  
    # A-Star done, traceback the 'previous' list to generate path to display

    if(not success):
        print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
        return

    a,b = -1,-1
    count = 0

    for i,d in enumerate(previous):
        if d['cur'] == [dim-1,dim-1]:
            a,b = previous[i].values()
            break

    while True:
        
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


    print('**********************\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')

    plt.clf()
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    printMap(map)


"""
Implements the fire spreading
"""
def fire_spread(map):

    dim = len(map[0])
    done = []
    justSpread = []

    
    """
    For each cell, check neighbord for fire and see if fire will spread to this cell
    """
    for x in range(dim):
        for y in range(dim):
            
            # Cell is already checked or is a wall
            if ([y,x] in done) or (map[y][x] == 1):
                continue

            done.append([y,x])

            neighborsOnFire = 0
            # Check neighbors of cell and see if they are on fire
            for i in range(4):
                if i is 0:
                    if (y+1 > dim-1) or (map[y+1][x] == 1) or ([y+1,x] in justSpread):
                        continue
                    elif (map[y+1][x] == 3):
                        neighborsOnFire += 1
                    else:
                        pass
                if i is 1:
                    if (x+1 > dim-1) or (map[y][x+1] == 1) or ([y,x+1] in justSpread):
                        continue
                    elif (map[y][x+1] == 3):
                        neighborsOnFire += 1
                    else:
                        pass
                if i is 2:
                    if (y-1 < 0) or (map[y-1][x] == 1) or ([y-1,x] in justSpread):
                        continue
                    elif (map[y-1][x] == 3):
                        neighborsOnFire += 1
                    else:
                        pass
                else:
                    if (x-1 < 0) or (map[y][x-1] == 1) or ([y,x-1] in justSpread):
                        continue
                    elif (map[y][x-1] == 3):
                        neighborsOnFire += 1
                    else:
                        pass

            if (neighborsOnFire == 0):
                continue
            else:
                spreadProb = abs(1 - ((1-FIREPROB)**neighborsOnFire))
                prob = np.random.choice(np.arange(0,2), p=[(1-spreadProb), spreadProb])

                # Turn cell into fire. Add to justSpread list to make sure we don't account for this new spot in later searches
                if(prob == 1):
                    map[y][x] = 3
                    justSpread.append([y,x])

    return map

if __name__ == '__main__':
    main()