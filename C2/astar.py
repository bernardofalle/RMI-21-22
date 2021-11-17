from heapq import *

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
        print(current)

        if current == goal:
            data = []
            c+=1
            while current in came_from:
                
                data.append(current)
                current = came_from[current]
            #data.append(start)
            print(c)
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
'''
array=[(2, 0), (4, 0), (6, 0), (6, 2), (8, 2), (10, 2), (12, 2), (14, 2), (14, 0), (14, -2), (16, -2), (16, -4), (18, -4), (20, -4), (20, -2), (18, -2), (18, 0), (16, 0), (16, 2), (18, 2), (20, 2), (22, 2), (24, 2), (24, 0), (24, -2), (24, -4)]
start=(6,2)
end=(2,2)
neighbours=[(0,2),(0,-2),(2,0),(-2,0)]

for i,j in neighbours:
            neigh=end[0]+i,end[1]+j
            if neigh in array:
                end=neigh

print(astar(array,start,end))
'''

