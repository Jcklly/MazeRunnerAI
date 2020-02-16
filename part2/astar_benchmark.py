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
DIM_G = 0

# Hold different values for the multiple DIM's to be able to draw chart
list1,list2,list3,list4,list5,list6,list7,list8,list9,list10 = [],[],[],[],[],[],[],[],[],[]

def main():

    global DIM_G
    MAP = []
    dim = 0

    try:
        dim = int(input("Please enter the DIM: "))
    except ValueError:
        print("Invalid Dim")
        exit()

    DIM_G = dim
    #MAP = create_map(dim, p)
    printMap(MAP, 0)


"""
Prints the Map
"""
def printMap(map, flag):

    cmap = colors.ListedColormap(['white', 'black', 'green'])
    bounds = [0,1,2]

    #plt.imshow(map, cmap=cmap, vmin=0,vmax=2)

    # Each list must have it's own value for matplotlib to correctly draw line plot
    x1,y1,x2,y2 = [],[],[],[]


    for d in list1:
        x1.append(d[0])
        y1.append(d[1])
    for d in list2:
        x2.append(d[0])
        y2.append(d[1])

    
    plt.plot(x1,y1,'o-', color='r', label = "Manhattan")
    plt.plot(x2,y2,'o-', color='g', label = "Euclidean")


    if flag is 0:
        plt.ylabel("Average Time")
        plt.title("Average Time vs P-Value")
    elif flag is 1:
        plt.ylabel("Average Expanded")
        plt.title("Average Expanded vs P-Value")
    else:
        plt.ylabel("Average Fringe Size")
        plt.title("Average Fringe vs P-Value")

    plt.xlabel("P-Value")
    #plt.axes([0,1,0,1])
    plt.legend(loc="upper right")

    benchAX = plt.axes([.2, 0.93, 0.15, 0.05])
    benchBtn = Button(benchAX, 'Time', color='red', hovercolor='green')
    benchBtn.on_clicked(benchmarkTime)

    benchAX1 = plt.axes([.4, 0.93, 0.15, 0.05])
    bench1Btn = Button(benchAX1, 'Expanded', color='red', hovercolor='green')
    bench1Btn.on_clicked(benchmarkExpanded)

    benchAX2 = plt.axes([.6, 0.93, 0.15, 0.05])
    bench2Btn = Button(benchAX2, 'Fringe', color='red', hovercolor='green')
    bench2Btn.on_clicked(benchmarkFringe)


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

        """
        # Check is to make sure it can get from start to goal
        check = False
        dim = len(map[0])
        check = dfs_algo(dim-1,dim-1,map)

        # Generate map that can be solved
        if (check == False):
            continue
        else:
            break
        """
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
def astar_man(map,flag):

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

    max_fringe = 0

    while len(open):

        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curG = cur[2]
        coord = cur[3]
        y = coord[0]
        x = coord[1]

        if (len(open)) > max_fringe:
            max_fringe = len(open)

        previous.append({'cur' : cur[3], 'prev' : cur[4]})

        # Found the end
        if(x == dim-1 and y == dim-1):
            heapq.heappush(open, cur)
            success = True
            break

        # Found wall, ignore and continue
        if(map[y][x] == 1):
            continue

        closed.append(coord)


        """
        print(str(cur))
        #print('------------')
        plt.clf()
        map[y][x] = 2
        map[0][0] = 2 
        map[dim-1][dim-1] = 2
        printMap(map)
        """
        

        # generate cells around
        # for each neighbor, check if in closed list or if already exist in open list.
        for i in range(4):
            if i is 0: # down

                # Already in closed list, skip it
                if ([y+1,x] in closed) or (y+1 > dim-1) or (map[y+1][x] == 1):
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
                if ([y,x+1] in closed) or (x+1 > dim-1) or (map[y][x+1] == 1):
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
                if ([y-1,x] in closed) or (y-1 < 0) or (map[y-1][x] == 1):
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
                if ([y,x-1] in closed) or (x-1 < 0) or (map[y][x-1] == 1):
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

    if flag is 0:
        return total_time
    elif flag is 1:
        return total_discovered
    elif flag is 2:
        return max_fringe

"""
Algorithm for A-Star using Euclidean distance
"""
def astar_euc(map,flag):

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

    max_fringe = 0

    while len(open):

        cur = heapq.heappop(open)
        curF = cur[0]
        curH = cur[1]
        curG = cur[2]
        coord = cur[3]
        y = coord[0]
        x = coord[1]

        if (len(open)) > max_fringe:
            max_fringe = len(open)

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

    if flag is 0:
        return total_time
    elif flag is 1:
        return total_discovered
    elif flag is 2:
        return max_fringe


def benchmarkTime(e):

    global list1
    global list2

    del list1[:]
    del list2[:]

    pTest = [0,0.1,0.2,0.3,0.4,0.5]
    
    # For each DIM starting at 10 and going up by 10, test it against a P-Value on a newly generated map 50 times
    for pv in pTest:
        totalTimeMan = 0
        totalTimeEuc = 0
        for i in range(50):
            m = create_map(DIM_G,pv)
            totalTimeMan += astar_man(m,0)
            totalTimeEuc += astar_euc(m,0)

        averageMan = totalTimeMan/50
        averageEuc = totalTimeEuc/50
        list1.append([pv,averageMan])
        list2.append([pv,averageEuc])  

        print("Done with p: ", pv)                           
 
    plt.clf()
    printMap(e,0)
    

def benchmarkExpanded(e):

    global list1
    global list2

    del list1[:]
    del list2[:]

    pTest = [0,0.1,0.2,0.3,0.4,0.5]
    
    # For each DIM starting at 10 and going up by 10, test it against a P-Value on a newly generated map 50 times
    for pv in pTest:
        totalTimeMan = 0
        totalTimeEuc = 0
        for i in range(50):
            m = create_map(DIM_G,pv)
            totalTimeMan += astar_man(m,1)
            totalTimeEuc += astar_euc(m,1)

        averageMan = totalTimeMan/50
        averageEuc = totalTimeEuc/50
        list1.append([pv,averageMan])
        list2.append([pv,averageEuc])  

        print("Done with p: ", pv)                           
 
    plt.clf()
    printMap(e,1)


def benchmarkFringe(e):

    global list1
    global list2

    del list1[:]
    del list2[:]

    pTest = [0,0.1,0.2,0.3,0.4,0.5]
    
    # For each DIM starting at 10 and going up by 10, test it against a P-Value on a newly generated map 50 times
    for pv in pTest:
        totalTimeMan = 0
        totalTimeEuc = 0
        for i in range(50):
            m = create_map(DIM_G,pv)
            totalTimeMan += astar_man(m,2)
            totalTimeEuc += astar_euc(m,2)

        averageMan = totalTimeMan/50
        averageEuc = totalTimeEuc/50
        list1.append([pv,averageMan])
        list2.append([pv,averageEuc])  

        print("Done with p: ", pv)                           
 
    plt.clf()
    printMap(e,2)


if __name__ == '__main__':
    main()