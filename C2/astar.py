from heapq import *
from mainRob import *

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,2),(0,-2),(2,0),(-2,0)]

    close_set = set()
    came_from = {}
    g = {start:0}
    f = {start:heuristic(start, goal)}
    oheap = []
    c=0
    heappush(oheap, (f[start], start))
    
    while oheap:

        current = heappop(oheap)[1]
        # print(current)

        if current == goal:
            data = []
            c+=1
            while current in came_from:
                
                data.append(current)
                current = came_from[current]
            #data.append(start)
            # print(c)
            return [start] + data[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = g[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < len(array):
                if 0 <= neighbor[1] < len(array[0]):                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= g.get(neighbor, 0):
                continue
                
            if  tentative_g_score < g.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                g[neighbor] = tentative_g_score
                f[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (f[neighbor], neighbor))
           
    return None

array=[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (6, 1), (6, 2), (7, 2), (8, 2), (9, 2), (10, 2), (11, 2), (12, 2), (13, 2), (14, 2), (14, 1), (14, 0), (14, -1), (14, -2), (15, -2), (16, -2), (16, -3), (16, -4), (17, -4), (18, -4), (19, -4), (20, -4), (20, -3), (20, -2), (19, -2), (18, -2), (18, -1), (18, 0), (17, 0), (16, 0), (16, 1), (16, 2), (17, 2), (18, 2), (19, 2), (20, 2), (21, 2), (22, 2), (23, 2), (24, 2), (24, 1), (24, 0), (24, -1), (24, -2), (24, -3), (24, -4), (24, -5), (24, -6), (24, -7), (24, -8), (24, -9), (24, -10), (23, -10), (22, -10), (21, -10), (20, -10), (19, -10), (18, -10), (17, -10), (16, -10), (15, -10), (14, -10), (13, -10), (12, -10), (11, -10), (10, -10), (9, -10), (8, -10), (7, -10), (6, -10), (5, -10), (4, -10), (3, -10), (2, -10), (1, -10), (0, -10), (-1, -10), (-2, -10), (-2, -9), (-2, -8), (-2, -7), (-2, -6), (-1, -6), (0, -6), (0, -5), (0, -4), (0, -3), (0, -2), (-1, -2), (-2, -2), (-2, -1), (-2, 0), (-1, 0)]


start=(0,0)
end=(18,-5)
neighbours=[(0,1),(0,-1),(1,0),(-1,0)]

for i,j in neighbours:
            neigh=end[0]+i,end[1]+j
            if neigh in array and neigh[0]%2==0 and neigh[1]%2==0:
                print(neigh)
                end=neigh

print(astar(array,start,end))


