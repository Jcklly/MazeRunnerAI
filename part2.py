import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import random
import copy
import math
import heapq
import time

# Global variable of the 2D array of the map
MAP = []

def main():

    global MAP
    dim = 0
    p = 0

    try:
        dim = int(input("Please enter the DIM: "))
        p = float(input("Please enter the P-Value: "))
    except ValueError:
        print("Invalid Dim/P-Value")
        exit()


    MAP = create_map(dim, p)
    printMap(MAP)


"""
Prints the Map
"""
def printMap(map):
    plt.imshow(map, cmap='gist_yarg', vmin=0,vmax=2)

    bfsAX = plt.axes([0.001, 0.7, 0.1, 0.05])
    bfsBtn = Button(bfsAX, 'BFS', color='red', hovercolor='green')
    bfsBtn.on_clicked(bfs_algo)

    dfsAX = plt.axes([0.001, 0.6, 0.1, 0.05])
    dfsBtn = Button(dfsAX, 'DFS', color='red', hovercolor='green')
    dfsBtn.on_clicked(dfs_algo)

    astar1AX = plt.axes([0.001, 0.5, 0.15, 0.05])
    astar1Btn = Button(astar1AX, 'A* Euclidean', color='red', hovercolor='green')
    astar1Btn.on_clicked(astar_euc)

    astar2AX = plt.axes([0.001, 0.4, 0.15, 0.05])
    astar2Btn = Button(astar2AX, 'A* Manhattan', color='red', hovercolor='green')
    astar2Btn.on_clicked(astar_man)

    plt.show()


"""
Function to generate and return a map (2D Array) given DIM and P
"""
def create_map(dim, p):

    map = [[0 for n in range(dim)] for n in range(dim)]

    for i in range(dim):
        for j in range(dim):
            #randomNum = random.randrange(0,2,1)
            prob = np.random.choice(np.arange(0,2), p=[(1-p), p])
            if(prob == 1):
                if((i == 0 and j == 0) or (i == dim-1 and j == dim-1)):
                    continue
                map[j][i] = 1


    return map



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
Algorithm for DFS
"""
def dfs_algo(e):

    map = copy.deepcopy(MAP)
    dim = len(MAP[0])
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
        if(x == dim-1 and y == dim-1):
            success = True
            break
        
        # Found wall, ignore and continue
        if(MAP[y][x] == 1):
            continue

        for i in range(4):
            if i is 1: # down
                if ([y+1,x] in discovered) or (y+1 > dim-1):
                    continue
                previous.append({'cur' : [y+1,x], 'prev' : [y,x]})
                stack.append([y+1,x])
                discovered.append([y+1,x])
                total_discovered += 1
            elif i is 2: # right
                if ([y,x+1] in discovered) or (x+1 > dim-1):
                    continue
                previous.append({'cur' : [y,x+1], 'prev' : [y,x]})
                stack.append([y,x+1])
                discovered.append([y,x+1])
                total_discovered += 1
            elif i is 3: # up
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


    end = time.process_time()
    total_time = end - start


    # DFS done, traceback the 'previous' list to generate path to display

    if(not success):
        print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: DFS\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
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


    print('**********************\n' + 'Algorithm: DFS\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')

    plt.clf()
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    printMap(map)




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
def astar_euc(e):

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

        """
        time.sleep(1)
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """

        closed.append(coord)

        # Found the end
        if(x == dim-1 and y == dim-1):
            heapq.heappush(open, cur)
            success = True
            break

        # Found wall, ignore and continue
        if(MAP[y][x] == 1):
            continue

        # generate cells around
        # for each neighbor, check if in closed list or if already exist in open list.
        for i in range(4):
            if i is 1: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (MAP[y+1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x,y+1,dim-1,dim-1)
                fScore = gScore + hScore

                previous.append({'cur' : [y+1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y+1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y+1,x], [y,x]))
                    total_discovered += 1
            elif i is 2: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (MAP[y][x+1] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x+1,y,dim-1,dim-1)
                fScore = gScore + hScore

                previous.append({'cur' : [y,x+1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x+1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y,x+1], [y,x]))
                    total_discovered += 1
            elif i is 3: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (MAP[y-1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x,y-1,dim-1,dim-1)
                fScore = gScore + hScore

                previous.append({'cur' : [y-1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y-1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
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
                if ([y,x-1] in closed) or (x-1 < 0) or (MAP[y][x-1] == 1):
                    continue

                gScore = curG + 1
                hScore = euclidean_distance(x-1,y,dim-1,dim-1)
                fScore = gScore + hScore
    
                previous.append({'cur' : [y,x-1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x-1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
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

    if(not success):
        print('**********************\n' + 'FAILED. Path not found.\n' + 'Algorithm: A-Star Euclidean\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: 0' + '\nTotal discovered: ' + str(total_discovered))
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


    print('**********************\n' + 'Algorithm: A-Star Euclidean\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')

    plt.clf()
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    printMap(map)



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


    while len(open):

        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curG = cur[2]
        coord = cur[3]
        y = coord[0]
        x = coord[1]

        """
        time.sleep(1)
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """

        closed.append(coord)

        # Found the end
        if(x == dim-1 and y == dim-1):
            heapq.heappush(open, cur)
            success = True
            break

        # Found wall, ignore and continue
        if(MAP[y][x] == 1):
            continue

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

                previous.append({'cur' : [y+1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y+1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y+1,x], [y,x]))
                    total_discovered += 1
            elif i is 2: # right

                # Already in closed list, skip it
                if ([y,x+1] in closed) or (x+1 > dim-1) or (MAP[y][x+1] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x+1,y,dim-1,dim-1)
                fScore = gScore + hScore

                previous.append({'cur' : [y,x+1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x+1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
                            open[i] = open[-1]
                            open.pop()
                            heapq.heapify(open)
                            heapq.heappush(open, tup)
                            break

                # Not found in open list. This check is needed otherwise we would could duplicates in the open list
                if(found == False):
                    heapq.heappush(open, (fScore, hScore, gScore, [y,x+1], [y,x]))
                    total_discovered += 1
            elif i is 3: # up

                # Already in closed list, skip it
                if ([y-1,x] in closed) or (y-1 < 0) or (MAP[y-1][x] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x,y-1,dim-1,dim-1)
                fScore = gScore + hScore

                previous.append({'cur' : [y-1,x], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y-1,x]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
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
                if ([y,x-1] in closed) or (x-1 < 0) or (MAP[y][x-1] == 1):
                    continue

                gScore = curG + 1
                hScore = manhattan_distance(x-1,y,dim-1,dim-1)
                fScore = gScore + hScore
    
                previous.append({'cur' : [y,x-1], 'prev' : [y,x]})

                # Check if it already in open list. Look at g-score and check if new value is less than old.
                # True would mean better path to the node has been found, update g-score, f-score and parent
                found = False
                for i,d in enumerate(open):
                    if d[3] == [y,x-1]:
                        found = True
                        if gScore < d[2]:
                            tup = (d[1] + gScore,d[1],gScore,d[3],[y,x])
                            print(tup)
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


    print('**********************\n' + 'Algorithm: A-Star Manhattan\n' + 'Time Taken: ' + str(total_time) + '\nPath Length: ' + str(count) + '\nTotal discovered: ' + str(total_discovered))
    print('**********************')

    plt.clf()
    map[0][0] = 2 
    map[dim-1][dim-1] = 2
    printMap(map)

if __name__ == '__main__':
    main()
