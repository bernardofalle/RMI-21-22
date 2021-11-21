#!/usr/bin/python3
from time import time
from mainRob import *

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f

    def __str__(self):
        return "Position: " + str(self.position) + "; G: " + str(self.g) + "; H: " + str(self.h) + "; F: " + str(self.f)
    

def astar(maze, start, end, start_time, timeout):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    i=27,13
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Sort
        open_list.sort()

        # Get the current node
        current_node = open_list[0]
        current_index = 0

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1], False # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[1]+i[1] > (len(maze) - 1) or node_position[1]+i[1] < 0 or node_position[0]+i[0] > (len(maze[len(maze)-1])-1) or node_position[0]+i[0] < 0:
                continue

            # Make sure walkable terrain
            if maze[i[1]-node_position[1]][node_position[0]+i[0]] != 'X':
                    continue

            # print('pos->'+str(node_position))

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = abs(child.position[0] - start_node.position[0])**2 + abs(child.position[1] - start_node.position[1])**2
            child.h = abs(child.position[0] - end_node.position[0])**2 + abs(child.position[1] - end_node.position[1])**2
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.f > open_node.f:
                    continue

            # Add the child to the open list
            # print('Child: ' + str(child.position))
            open_list.append(child)

        # Time out function
        if time() - start_time > timeout:
            return [], True
