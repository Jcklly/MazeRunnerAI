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
FIREPROB = 0.0
FIRESTART = [0,0]
FIRELIST = []
ONFIRE = []
DIM_G = 0.0
P_G = 0.0

# Average success vs q
graph1 = []
graph2 = []
graph3 = []

def main():

    #global MAP
    MAP = []
    global FIREPROB
    global DIM_G
    global P_G
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
    DIM_G = dim
    P_G = p
    MAP = create_map(dim, p)
    printMap(MAP)


"""
Prints the Map
"""
def printMap(map):

    cmap = colors.ListedColormap(['white', 'black', 'green', 'red'])
    bounds = [0,1,2,3]

    #plt.imshow(map, cmap=cmap, vmin=0,vmax=3)

    x1 = []
    y1 = []
    x2 = []
    y2 = []
    x3 = []
    y3 = []
    for d in graph1:
        x1.append(d[0])
        y1.append(d[1])

    for d in graph2:
        x2.append(d[0])
        y2.append(d[1])

    for d in graph3:
        x3.append(d[0])
        y3.append(d[1])

    plt.plot(x1,y1,'o-', color='black', label="Strategy 1")
    plt.plot(x2,y2,'o-', color='red', label="Strategy 2")
    plt.plot(x3,y3,'o-', color='green', label="Strategy 3")

    plt.ylabel("Average")
    plt.xlabel("Q-Value")
    plt.title("Success vs Flammability Q")
    plt.legend(loc="upper right")
    #plt.axes([0,1,0,1])

    bench1 = plt.axes([0.3, 0.94, 0.05, 0.05])
    bench1Btn = Button(bench1, 'S1', color='red', hovercolor='green')
    bench1Btn.on_clicked(bench_strat1)

    bench2 = plt.axes([0.5, 0.94, 0.05, 0.05])
    bench2Btn = Button(bench2, 'S2', color='red', hovercolor='green')
    bench2Btn.on_clicked(bench_strat2)

    s3 = plt.axes([0.7, 0.94, 0.05, 0.05])
    s3Btn = Button(s3, 'S3', color='red', hovercolor='green')
    s3Btn.on_clicked(strat3)

    plt.show()

def printPath(map):
    cmap = colors.ListedColormap(['white', 'black', 'green', 'red'])
    bounds = [0,1,2,3]

    plt.imshow(map, cmap=cmap, vmin=0,vmax=3)

    bench1 = plt.axes([0.001, 0.5, 0.05, 0.05])
    bench1Btn = Button(bench1, 'S1', color='red', hovercolor='green')
    bench1Btn.on_clicked(bench_strat1)

    bench2 = plt.axes([0.001, 0.4, 0.05, 0.05])
    bench2Btn = Button(bench2, 'S2', color='red', hovercolor='green')
    bench2Btn.on_clicked(bench_strat2)

    s3 = plt.axes([0.001, 0.3, 0.05, 0.05])
    s3Btn = Button(s3, 'S3', color='red', hovercolor='green')
    s3Btn.on_clicked(strat3)
    plt.show()


"""
Function to generate and return a map (2D Array) given DIM and P.
Must be able to reach fire spot from beginning and be able to be completed.
"""
def create_map(dim, p):

    global FIRESTART
    global FIRELIST
    global ONFIRE
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
        if (fy == 0 and fx == 0) or (fy == dim-1 and fx == dim-1):
            continue
        map[fy][fx] = 3

        # Check1 is to make sure it can get from start to goal
        # Check2 is to make sure it can get from start to fire
        check1 = False
        check2 = False
        dim = len(map[0])
        check1 = dfs_algo(dim-1,dim-1, map)
        check2 = dfs_algo(fy,fx,map)

        # Generate map that can be solved
        if ((check1 == False) or (check2 == False)):
            continue
        else:
            break

    FIRESTART = [fy,fx]
    ONFIRE.append([fy,fx])
    # Add nieghtbors of fire location for spreading

    if (fy+1 > dim-1) or (map[fy+1][fx] == 1) or ([fy+1,fx] in FIRELIST):
        pass
    else:
        FIRELIST.append([fy+1,fx])
    if (fx+1 > dim-1) or (map[fy][fx+1] == 1) or ([fy,fx+1] in FIRELIST):
        pass
    else:
        FIRELIST.append([fy,fx+1])
    if (fy-1 < 0) or (map[fy-1][fx] == 1) or ([fy-1,fx] in FIRELIST):
        pass
    else:
        FIRELIST.append([fy-1,fx])
    if (fx-1 < 0) or (map[fy][fx-1] == 1) or ([fy,fx-1] in FIRELIST):
        pass
    else:
        FIRELIST.append([fy,fx-1])

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
Algorithm for A-Star using Manhattan distance. Used for Strategy 1
"""
def astar_man_strat1(map):

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

        """
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """

        previous.append({'cur' : cur[3], 'prev' : cur[4]})

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
            if i is 0: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (map[y+1][x] == 1) or (map[y+1][x] == 3):
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
            elif i is 1: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (map[y][x+1] == 1) or (map[y][x+1] == 3):
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
            elif i is 2: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (map[y-1][x] == 1) or (map[y-1][x] == 3):
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
                if ([y,x-1] in closed) or (x-1 < 0) or (map[y][x-1] == 1) or (map[y][x-1] == 3):
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
 
        # fire spreads
        map = fire_spread(map)

        
    end = time.process_time()
    total_time = end - start
  
    # A-Star done, traceback the 'previous' list to generate path to display

    if(not success):
        #print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
        return False

    
    a,b = -1,-1
    count = 0

    for i,d in enumerate(previous):
        if d['cur'] == [dim-1,dim-1]:
            a,b = previous[i].values()
            break

    while True:
        
        prevX = b[1]
        prevY = b[0]

        if (map[prevY][prevX] == 3):
            return False

        if (prevY is 0) and (prevX is 0):
            break

        map[prevY][prevX] = 2
        count += 1

        for i, d in enumerate(previous):
            if d['cur'] == [prevY,prevX]:
                a,b = previous[i].values()
                break

    return True
    """
    print('**********************\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')

    plt.clf()
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    printMap(map)
    """
    
"""
Algorithm for A-Star using Manhattan distance - Strategy 2
"""
def astar_man_strat2(map):

    #map = copy.deepcopy(MAP)

    dim = len(map[0])
    total_discovered = 0

    success = False

    start = time.process_time()

    # Priority queue
    open = []
    previous = []
    heapq.heapify(open)

    fire_location = FIRESTART
    fy = fire_location[0]
    fx = fire_location[1]

    closed = []

    preH = euclidean_distance(0,0,dim-1,dim-1)
    fireH = euclidean_distance(0,0,fx,fy)
   
    # ( f value, h value, fire value, g value, coordinate on map, parent)
    heapq.heappush(open, (preH-fireH, preH, fireH, 0, [0,0], [0,0]))

    #print(map)

    while len(open):
    
        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curFire = cur[2]
        curG = cur[3]
        coord = cur[4]
        y = coord[0]
        x = coord[1]

        """
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """

        previous.append({'cur' : cur[4], 'prev' : cur[5]})

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
            if i is 0: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (map[y+1][x] == 1) or (map[y+1][x] == 3):
                    continue

                fireScore = closestFire(y+1,x)

                gScore = curG + 1
                hScore = euclidean_distance(x,y+1,dim-1,dim-1)
                #fireScore = manhattan_distance(x,y+1,fx,fy)
                fScore = (hScore - fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y+1,x]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y+1,x], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                else:
                    pass
            elif i is 1: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (map[y][x+1] == 1) or (map[y][x+1] == 3):
                    continue

                fireScore = closestFire(y,x+1)

                gScore = curG + 1
                hScore = euclidean_distance(x+1,y,dim-1,dim-1)
                #fireScore = manhattan_distance(x+1,y,fx,fy)
                fScore = (hScore - fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y,x+1]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y,x+1], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
            elif i is 2: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (map[y-1][x] == 1) or (map[y-1][x] == 3):
                    continue

                fireScore = closestFire(y-1,x)

                gScore = curG + 1
                hScore = euclidean_distance(x,y-1,dim-1,dim-1)
                #fireScore = manhattan_distance(x,y-1,fx,fy)
                fScore = (hScore - fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y-1,x]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y-1,x], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y-1,x], 'prev' : [y,x]})
            else: # left

                # Already in closed list, skip it
                if ([y,x-1] in closed) or (x-1 < 0) or (map[y][x-1] == 1) or (map[y][x-1] == 3):
                    continue

                fireScore = closestFire(y,x-1)

                gScore = curG + 1
                hScore = euclidean_distance(x-1,y,dim-1,dim-1)
                #fireScore = manhattan_distance(x-1,y,fx,fy)
                fScore = (hScore - fireScore)
                    
                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y,x-1]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y,x-1], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y,x-1], 'prev' : [y,x]})
 
        # fire spreads
        map = fire_spread(map)

        
    end = time.process_time()
    total_time = end - start
  
    # A-Star done, traceback the 'previous' list to generate path to display

    if(not success):
        #print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
        return False

    
    a,b = -1,-1
    count = 0

    for i,d in enumerate(previous):
        if d['cur'] == [dim-1,dim-1]:
            a,b = previous[i].values()
            break

    while True:
        
        prevX = b[1]
        prevY = b[0]

        if (map[prevY][prevX] == 3):
            return False

        if (prevY is 0) and (prevX is 0):
            break

        map[prevY][prevX] = 2
        count += 1

        for i, d in enumerate(previous):
            if d['cur'] == [prevY,prevX]:
                a,b = previous[i].values()
                break

    return True



"""
Returns the closest distance to the nearest fire of the current node
"""
def closestFire(y,x):

    fy = FIRESTART[0]
    fx = FIRESTART[1]
    closest = euclidean_distance(x,y,fx,fy)
    for i,d in enumerate(ONFIRE):
        fy = d[0]
        fx = d[1]

        distance = euclidean_distance(x,y,fx,fy)
        if(distance <= closest):
            closest = distance

    return closest


"""
Implements the fire spreading
"""
def fire_spread(map):

    global ONFIRE
    dim = len(map[0])
    done = []
    justSpread = []
    
    """
    For each cell in FIRELIST, check neighbord for fire and see if fire will spread to this cell. Adjust list appropriately
    """
    i = 0
    size = len(FIRELIST)
    tempList = []
    probList = []

    while(i < size):
        
        if(len(FIRELIST) == 0):
            break
        cur = FIRELIST.pop(0)
        y = cur[0]
        x = cur[1]

        del tempList[:]

        neighborsOnFire = 0
        # Check if neighbors are on fire. If so add to count
        if (y+1 > dim-1) or (map[y+1][x] == 1) or ([y+1,x] in justSpread):
            pass
        elif (map[y+1][x] == 3):
            neighborsOnFire += 1
        else:
            if [y+1,x] not in FIRELIST:
                tempList.append([y+1,x])

        if (x+1 > dim-1) or (map[y][x+1] == 1) or ([y,x+1] in justSpread):
            pass
        elif (map[y][x+1] == 3):
            neighborsOnFire += 1
        else:
            if [y,x+1] not in FIRELIST:
                tempList.append([y,x+1])

        if (y-1 < 0) or (map[y-1][x] == 1) or ([y-1,x] in justSpread):
            pass
        elif (map[y-1][x] == 3):
            neighborsOnFire += 1
        else:
            if [y-1,x] not in FIRELIST:
                tempList.append([y-1,x])

        if (x-1 < 0) or (map[y][x-1] == 1) or ([y,x-1] in justSpread):
            pass
        elif (map[y][x-1] == 3):
            neighborsOnFire += 1
        else:
            if [y,x-1] not in FIRELIST:
                tempList.append([y,x-1])

        spreadProb = abs(1 - ((1.0-FIREPROB) ** neighborsOnFire))

        #prob = np.random.choice([0,1], p=[(1.0-spreadProb), spreadProb])
        #print(str(cur) + " : " + str(spreadProb) + " : " + str(neighborsOnFire) + " : " + str(probList))
        #print(str(prob))
        prob = -1
        if random.uniform(0,1) <= spreadProb:
            prob = 1
        else:
            prob = 0


        # Turn cell into fire. Add to justSpread list to make sure we don't account for this new spot in later searches
        if(prob == 1):
            justSpread.append([y,x])
            ONFIRE.append([y,x])
            map[y][x] = 3
            FIRELIST.extend(tempList)
        else:
            FIRELIST.append([y,x])

        i += 1

    return map

"""
returns the quadrant ont he map of the given coordinate
"""
def findQuadrant(coord, dim):
    y = coord[0]
    x = coord[1]

    if (int((dim/2)) <= x) and (int((dim/2)) >= y):
        return 0
    elif (int((dim/2)) >= x) and (int((dim/2)) >= y):
        return 1
    elif (int((dim/2)) >= x) and (int((dim/2)) <= y):
        return 2
    else:
        return 3

"""
down-right: a,b,c,d = 3,2,1,0
right-down: a,b,c,d = 2,3,1,0
"""



"""
Algorithm for A-Star using Manhattan distance - Strategy 2
"""
def astar_man_strat3(map):

    #map = copy.deepcopy(MAP)

    dim = len(map[0])
    total_discovered = 0

    success = False

    start = time.process_time()

    # Priority queue
    open = []
    previous = []
    heapq.heapify(open)

    fire_location = FIRESTART
    fy = fire_location[0]
    fx = fire_location[1]

    closed = []

    preH = euclidean_distance(0,0,dim-1,dim-1)
    fireH = euclidean_distance(0,0,fx,fy)
   
    # ( f value, h value, fire value, g value, coordinate on map, parent)
    heapq.heappush(open, (preH-fireH, preH, fireH, 0, [0,0], [0,0]))

    #print(map)

    while len(open):

        onFireCopy = copy.deepcopy(ONFIRE)
        fireListCopy = copy.deepcopy(FIRELIST)
        futureMap = copy.deepcopy(map)
    
        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curFire = cur[2]
        curG = cur[3]
        coord = cur[4]
        y = coord[0]
        x = coord[1]


        """
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printPath(map)
        """

        previous.append({'cur' : cur[4], 'prev' : cur[5]})

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

        while True:
            fCord, fValue = closestFireFuture(y,x,onFireCopy)
            if int(fValue) <= 10:
                break
            futureMap, fireListCopy, onFireCopy = fire_spread_future(futureMap, fireListCopy, onFireCopy, coord, closed)


        # generate cells around
        # for each neighbor, check if in closed list or if already exist in open list.
        for i in range(4):
            if i is 0: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (map[y+1][x] == 1) or (map[y+1][x] == 3):
                    continue

                fCord, fireScore = closestFireFuture(y+1,x,onFireCopy)

                gScore = curG + 1
                hScore = euclidean_distance(x,y+1,dim-1,dim-1)
                #fireScore = manhattan_distance(x,y+1,fx,fy)
                fScore = (hScore - fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y+1,x]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y+1,x], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                else:
                    pass
            elif i is 1: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (map[y][x+1] == 1) or (map[y][x+1] == 3):
                    continue

                fCord, fireScore = closestFireFuture(y,x+1,onFireCopy)

                gScore = curG + 1
                hScore = euclidean_distance(x+1,y,dim-1,dim-1)
                #fireScore = manhattan_distance(x+1,y,fx,fy)
                fScore = (hScore -fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y,x+1]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y,x+1], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
            elif i is 2: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (map[y-1][x] == 1) or (map[y-1][x] == 3):
                    continue

                fCord, fireScore = closestFireFuture(y-1,x,onFireCopy)

                gScore = curG + 1
                hScore = euclidean_distance(x,y-1,dim-1,dim-1)
                #fireScore = manhattan_distance(x,y-1,fx,fy)
                fScore = (hScore - fireScore)

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y-1,x]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y-1,x], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y-1,x], 'prev' : [y,x]})
            else: # left

                # Already in closed list, skip it
                if ([y,x-1] in closed) or (x-1 < 0) or (map[y][x-1] == 1) or (futureMap[y][x-1] == 3):
                    continue

                fCord, fireScore = closestFireFuture(y,x-1,onFireCopy)

                gScore = curG + 1
                hScore = euclidean_distance(x-1,y,dim-1,dim-1)
                #fireScore = manhattan_distance(x-1,y,fx,fy)
                fScore = (hScore - fireScore)
                    
                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[4] == [y,x-1]:
                        found = True
                        if gScore < d[3]:
                            tup = (d[1] - d[2],d[1],fireScore,gScore,d[4],[y,x])
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, fireScore, gScore, [y,x-1], [y,x]))
                    total_discovered += 1
                    #previous.append({'cur' : [y,x-1], 'prev' : [y,x]})
 
        # fire spreads
        map = fire_spread(map)

        
    end = time.process_time()
    total_time = end - start
  
    # A-Star done, traceback the 'previous' list to generate path to display

    if(not success):
        #print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
        return False

    
    a,b = -1,-1
    count = 0

    for i,d in enumerate(previous):
        if d['cur'] == [dim-1,dim-1]:
            a,b = previous[i].values()
            break

    while True:
        
        prevX = b[1]
        prevY = b[0]

        if (map[prevY][prevX] == 3):
            return False

        if (prevY is 0) and (prevX is 0):
            break

        map[prevY][prevX] = 2
        count += 1

        for i, d in enumerate(previous):
            if d['cur'] == [prevY,prevX]:
                a,b = previous[i].values()
                break

    return True




"""
Implements the fire spreading
"""
def fire_spread_future(map, fireList, onFire, coord, closedList):

    dim = len(map[0])
    done = []
    justSpread = []
    
    """
    For each cell in FIRELIST, check neighbord for fire and see if fire will spread to this cell. Adjust list appropriately
    """
    i = 0
    size = len(fireList)
    tempList = []
    probList = []

    while(i < size):
        
        if(len(fireList) == 0):
            break
        cur = fireList.pop(0)
        y = cur[0]
        x = cur[1]

        del tempList[:]

        neighborsOnFire = 0
        # Check if neighbors are on fire. If so add to count
        if (y+1 > dim-1) or (map[y+1][x] == 1) or ([y+1,x] in justSpread):
            pass
        elif (map[y+1][x] == 3):
            neighborsOnFire += 1
        else:
            if [y+1,x] not in fireList:
                tempList.append([y+1,x])

        if (x+1 > dim-1) or (map[y][x+1] == 1) or ([y,x+1] in justSpread):
            pass
        elif (map[y][x+1] == 3):
            neighborsOnFire += 1
        else:
            if [y,x+1] not in fireList:
                tempList.append([y,x+1])

        if (y-1 < 0) or (map[y-1][x] == 1) or ([y-1,x] in justSpread):
            pass
        elif (map[y-1][x] == 3):
            neighborsOnFire += 1
        else:
            if [y-1,x] not in fireList:
                tempList.append([y-1,x])

        if (x-1 < 0) or (map[y][x-1] == 1) or ([y,x-1] in justSpread):
            pass
        elif (map[y][x-1] == 3):
            neighborsOnFire += 1
        else:
            if [y,x-1] not in fireList:
                tempList.append([y,x-1])

        spreadProb = abs(1 - ((1.0-FIREPROB) ** neighborsOnFire))

        #prob = np.random.choice([0,1], p=[(1.0-spreadProb), spreadProb])
        #print(str(cur) + " : " + str(spreadProb) + " : " + str(neighborsOnFire) + " : " + str(probList))
        #print(str(prob))
        prob = -1
        if random.uniform(0,1) <= spreadProb:
            prob = 1
        else:
            prob = 0


        # Turn cell into fire. Add to justSpread list to make sure we don't account for this new spot in later searches
        if(prob == 1):
            justSpread.append([y,x])
            onFire.append([y,x])
            map[y][x] = 3
            fireList.extend(tempList)
        else:
            fireList.append([y,x])

        i += 1

    return map,fireList,onFire

"""
Returns the closest coordinate of the nearest cell on fire
"""
def closestFireFuture(y,x, onFire):

    finalX, finalY = 0,0

    fy = FIRESTART[0]
    fx = FIRESTART[1]
    closest = euclidean_distance(x,y,fx,fy)
    for i,d in enumerate(onFire):
        fy = d[0]
        fx = d[1]

        distance = euclidean_distance(x,y,fx,fy)
        if(distance <= closest):
            closest = distance
            finalX = fx
            finalY = fy

    return [finalY,finalX], closest


def bench_strat1(e):
    global FIRELIST
    global ONFIRE
    global graph1
    global FIREPROB
    del graph1[:]
    print("Running Strategy 1 50 times for success rate")
    l = []
    q = [0.1,0.2,0.3,0.4]
    for qc in q:
        FIREPROB = qc
        del l[:]
        for i in range(50):
            del FIRELIST[:]
            del ONFIRE[:]
            m = create_map(DIM_G, P_G)
            check = astar_man_strat1(m)
            l.append(check)
            print("Test ", i+1, " complete for ", qc, " ... ", check)

        average = l.count(True)/50
        graph1.append([qc,average])
    print(l.count(True), " : ", l.count(False))
    print(graph1)
    plt.clf()
    printMap(map)


def bench_strat2(e):
    
    global FIRELIST
    global ONFIRE
    global graph2
    global FIREPROB
    del graph2[:]
    print("Running Strategy 2 50 times for success rate")
    l = []
    q = [0.1,0.2,0.3,0.4]
    for qc in q:
        FIREPROB = qc
        del l[:]
        for i in range(50):
            del FIRELIST[:]
            del ONFIRE[:]
            m = create_map(DIM_G, P_G)
            check = astar_man_strat2(m)
            l.append(check)
            print("Test ", i+1, " complete for ", qc, " ... ", check)

        average = l.count(True)/50
        graph2.append([qc,average])
    print(l.count(True), " : ", l.count(False))
    print(graph2)
    plt.clf()
    printMap(map)



def strat3(e):

    
    global FIRELIST
    global ONFIRE
    global graph3
    global FIREPROB
    
    del graph3[:]
    print("Running Strategy 3 50 times for success rate")
    l = []
    q = [0.1,0.2,0.3,0.4]
    for qc in q:
        FIREPROB = qc
        del l[:]
        for i in range(50):
            del FIRELIST[:]
            del ONFIRE[:]
            m = create_map(DIM_G, P_G)
            check = astar_man_strat3(m)
            l.append(check)
            print("Test ", i+1, " complete for ", qc, " ... ", check)

        average = l.count(True)/50
        graph3.append([qc,average])
    print(l.count(True), " : ", l.count(False))
    print(graph3)
    plt.clf()
    printMap(map)
    
# [[0.1, 0.54], [0.2, 0.04], [0.3, 0.0], [0.4, 0.0]]

if __name__ == '__main__':
    main()