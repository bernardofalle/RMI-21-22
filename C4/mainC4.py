#!/usr/bin/python3
import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from math import inf
from astar import *
import logging
import numpy as np
import itertools

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.posList = []
        self.errList = []
        self.counter = 0
        self.counter2 = 0
        self.countergps = 0
        self.counterfree = 0
        self.length = 2
        self.lengthrot = 2
        self.objective = 0
        self.endCycle = True
        self.onRot = False
        self.minus = False
        self.South = False
        self.maze = Lab()
        self.last_x = 27
        self.last_y = 13
        self.unknown = []
        self.known = []
        self.path = []
        self.searching = False
        self.walked = [(0, 0)]
        self.pathfollowing = False
        self.haspath = False
        self.beacon_coordinates = [(0, 0)]
        self.beacon_nums = []
        self.go_to_beacons = False
        self.f = None
        self.final_path = []
        self.estimated_velocity = [(0, 0)]
        self.pose = [(0, 0)]
        self.WallClose = False
        self.side_correction = False
        self.left_detected = False
        self.right_detected = False
        self.go0 = False
        self.first = True


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):
        """
        The main function of the program. Call every other function and chooses between them.
        :return:
        """
        # TODO Remove this on delivery
        # Remove absolute gps coordinates
        self.gpsConverter()
        self.real_x = self.measures.x
        self.real_y = self.measures.y
        self.measures.x = self.pose[-1][0]
        self.measures.y = self.pose[-1][1]
        threshold_warn = 0.5
        threshold_error = 1
        threshold_critical = 2
        difference_x = round(self.measures.x - self.real_x, 3)
        difference_y = round(self.measures.y - self.real_y, 3)
        difference_distance = round(math.sqrt(difference_y ** 2 + difference_x ** 2), 3)
        center_sensor = self.measures.irSensor[0]
        left_sensor = self.measures.irSensor[1]
        right_sensor = self.measures.irSensor[2]
        back_sensor = self.measures.irSensor[3]
        if not self.onRot:
            logging.debug(f'The real GPS values are (X,Y): ({round(self.real_x, 3)},{round(self.real_y, 3)})')
            logging.debug(f'The calculates GPS values are: ({round(self.measures.x, 3)},{round(self.measures.y, 3)})')
            logging.debug(f'The difference between values is ({difference_x},{difference_y})')
            logging.debug(f'The values of the sensors are (f, l, r, b): {center_sensor, left_sensor, right_sensor, back_sensor}')
            if difference_distance >= threshold_critical:
                logging.critical(f'Gigantic error experienced, larger than {threshold_critical}')
            elif difference_distance >= threshold_error:
                logging.error(f'Enormous error experienced, larger than {threshold_error}')
            elif difference_distance >= threshold_warn:
                logging.warning(f'A lot of error experienced, larger than {threshold_warn}')

        # Check if the compass is facing south
        self.checkChangeCompass()

        # If it is facing south, offset the compass
        if self.South and self.measures.compass < -90:
            self.measures.compass += 360
        # If it has travelled a distance of 2
        if self.endCycle:
            if not self.onRot:
                self.converter(0, 0)
                self.left_detected = left_sensor >= 1.5
                self.right_detected = right_sensor >= 1.5
                # if (self.distance(left_sensor) + self.distance(right_sensor) >= 2) and (left_sensor <= 0.6 or right_sensor <= 0.6):
                #     self.side_correction = False
                # else:
                #     self.side_correction = True
                logging.info(f'Cycle ended on {round(self.measures.x), round(self.measures.y)}')
                logging.info(f'Wall on Left: {self.left_detected}, Wall on Right: {self.right_detected}')
            # If you are rotating
            if self.onRot:
                self.left_detected = False
                self.right_detected = False
                logging.debug(f'Rotating to {self.objective}')
                # Start rotating to the predefined. Once it is done, this function returns false
                self.onRot = self.rotate(0.5, 0, 0, self.objective, False)

            # If it is following a path and needs to locate the next position
            elif self.searching:
                logging.info('Following path... ')
                # Find the current location of the robot
                loc = self.round_even(self.measures.x), self.round_even(self.measures.y)

                # If you are on the first member of path, remove it
                if loc == self.path[0]:
                    self.path = self.path[1:]
                # If the path has ended, reset the variables
                if len(self.path) == 0:
                    self.haspath = False
                    self.searching = False
                else:
                    # Calculate the difference between the current location and the next position of the path
                    x, y = (self.path[0][0] - loc[0]), (self.path[0][1] - loc[1])

                    # Get the current corrected compass orientation
                    current = self.corrCompass()

                    # With the difference between coordinates, it can determine where it should be oriented to
                    if x < 0:
                        self.objective = 180
                    elif x > 0:
                        self.objective = 0
                    elif y < 0:
                        self.objective = -90
                    elif y > 0:
                        self.objective = 90
                    else:
                        # If the differences are (0,0), the next position in path is the current one,
                        # so does not rotate.
                        self.onRot = False
                    logging.info(f'To go from {loc} to {self.path[0]}, it needs to go in {self.objective}')
                    if self.objective != current:
                        # If the the objective orientation is different then the current one, rotate to it
                        self.onRot = True
                    else:
                        # If not, do not rotate, reset the searching variable (so it can move in front) and remove the
                        # next path coordinate
                        self.onRot = False
                        self.searching = False
                        # self.path = self.path[1:]

            # If it is not following a path and there is an obstacle close to the front of the agent
            elif (center_sensor + back_sensor)/2 >= 1.0 and not self.pathfollowing:
                # Search its surroundings for an available path and rotates to it
                logging.info('Cannot walk in front, checking sides...')
                self.searchUnknown()
                self.whosFree()
                self.onRot = True
                self.first = False
            elif self.first:
                if self.corrCompass() == 0:
                    self.onRot = True
                    self.objective = 180
                    self.searchUnknown()
                elif self.corrCompass() == 180:
                    self.onRot = True
                    self.objective = 0
                    self.searchUnknown()
                    self.first = 0
            else:
                if not self.onRot:
                    logging.debug(f'Ended cycle on ({self.measures.x},{self.measures.y})')
                # If it doesn't have anything in front, nor is in middle of a rotation, nor is it searching for a next
                # position, add the walked, known and unknown coordinates
                self.appendWalked()
                self.amknown = self.searchKnown()
                self.searchUnknown()
                logging.debug(f'I have these coordinates as known: {self.known}')
                logging.debug(f'I have these coordinates as unknown: {self.unknown}')

                # If it was facing South, reset the variable (it will be checked later)
                if self.South:
                    self.South = False

                # If it is on an already known coordinate
                if self.amknown:
                    logging.info('I am on a known cell')
                    # Starts the searching variable
                    self.searching = True

                    # If it does not have a path
                    if not self.haspath:
                        # Get the current coordinates
                        start = self.round_even(self.measures.x), self.round_even(self.measures.y)
                        #if self.go0 :
                        #    self.path, timeout = astar(self.maze.matrix, start, (0,0), time(), 0.5)
                        #    print(self.path)
                        # Define the list of possible ends
                        end_list = self.unknown

                        # From the list of possible ends and the current coordinates, search the smallest path
                        end = self.a(start, end_list)
                        if not end and self.go0:
                            end = self.a(start, [(-1,0)])
                        # Remove the odd cells from the path, add the end and removing the start
                        self.path = [items for items in self.path if items[0] % 2 == 0 and items[1] % 2 == 0]
                        self.path.append((2 * end[0] - self.path[-1][0], 2 * end[1] - self.path[-1][1]))
                        self.path.remove(start)

                        # Start the variables
                        self.pathfollowing = True
                        self.haspath = True
                        logging.info(f'I calculated a path which is: {self.path}')
                    else:
                        # If it has a path already, walk in front
                        self.endCycle = False
                        logging.info(f'I already have a path, which is {self.path}')

                # If it has not following paths, move in front
                if not self.pathfollowing:
                    self.endCycle = False
                    logging.info(f'I am not following paths')

        else:
        # If it is not in the end of a cycle, move in front
            self.endCycle = self.moveFront(0.1, 0.01, 0.00005)
            logging.debug(f'I am moving in front and facing {self.measures.compass} ({self.corrCompass()}, with the '
                          f'objective {self.obj})')
        
    def a(self, start, goal_list):
        """
                The start of an a start algorithm, performing the needed operations before the algorithm is started
                :param start: Coordinate
                :param goal_list: List of coordinates
                :return:
                """

        # Defining variables
        min_len = inf
        min_idx = -1
        min_path = []

        # For all the possible goals
        for idx, goal in enumerate(goal_list):
            # Defining the possible neighbours, so it can land on an even space
            neighbours = [(0, 1), (0, -1), (1, 0), (-1, 0)]

            # For every neighbours
            for i, j in neighbours:
                # Calculating the neighbour coordinates
                neigh = goal[0] + i, goal[1] + j

                # If the neighbour is free and even
                if self.maze.matrix[13 - neigh[1]][neigh[0] + 27] == 'X' and neigh[0] % 2 == 0 and neigh[1] % 2 == 0:
                    # Calculate the path
                    final_goal = neigh
                    if start and final_goal:
                        self.path, timeout = astar(self.maze.matrix, start, final_goal, time(), 0.5)
                    else:
                        continue

                    # If the length of the current path is smaller then the current minimum, it becomes the minimum
                    length = len(self.path)
                    if length < min_len:
                        min_idx = idx
                        min_len = length
                        min_path = self.path
        # The path is equal to the minimum path
        self.path = min_path

        # If the goal list isn't empty
        if goal_list:
            # Returns the end of the path
            end = goal_list[min_idx]
            return end
        else:
            # After mapping the whole map, calculate the paths between beacons and prints the path
            #print('FULL MAPPING DONE ')
            if self.go0:
                self.finish()
                sys.exit()
            I = (0,0)
            self.beacon_coordinates.remove(I)
            perms = list(itertools.permutations(self.beacon_coordinates))
            path = []
            k = []
            for i in range(0, len(perms)):
                k.append(list(perms[i]))
            for e in k:
                e.insert(0, I)
                e.append(I)
            
            perms = k

            for perm in perms:
                for i in range(0, len(perm) - 1):
                    print(perm, i)
                    try:
                        p, timeout = astar(self.maze.matrix, perm[i], perm[i + 1], time(), 2)
                    except:
                        p = [0 for number in range(50)]
                    print(p)
                    path.extend(p)
                    path.pop()
                if len(self.final_path) == 0:
                    print(0)
                    self.final_path = path
                elif len(path) <= len(self.final_path):
                    print(1)
                    self.final_path = path
                path = []
            self.final_path.append((0,0))
            print(self.path)
            self.final_path = [i for i in self.final_path if i[0] % 2 == 0 and i[1] % 2 == 0]

            self.writePath()
            
            self.maze.matrix[13][27] = '0'
            if self.beacon_coordinates :
                for e in self.beacon_coordinates:
                    y = 13 - e[1]
                    x = 27 + e[0]
                    self.maze.matrix[y][x] = str(self.beacon_nums[0])
                    self.beacon_nums = self.beacon_nums[1:]
            self.writeMap()
            self.go0 = True

    def writeMap(self):
        """
        Converts the map matrix to a .map file
        :return:
        """
        # Opens the file
        f = open(self.f + '.map', 'w+')

        # For every element in the matrix, writes it in the file
        for line in self.maze.matrix:
            for element in line:
                f.write(element)
            f.write('\n')
        f.close()
    
    def writePath(self):
        """
        Writes the path from a list to a .out file
        :return:
        """
        # Defines variable and opens the file
        i = 0
        f = open(self.f + '.path', 'w+')
        # For every tuple in the list, writes it to the file
        for x, y in self.final_path:
            f.write(str(x) + ' ' + str(y) + ' ')

            # If the tuple is a beacon, annotate it
            if (x, y) in self.beacon_coordinates and (x, y) != (0, 0):
                f.write('#' + str(i))
                i += 1
            f.write('\n')
        f.close()

    def moveFront(self, Kp, Kd, Ki):
        """
                PID for moving in front
                :param Kp:
                :param Kd:
                :param Ki:
                :return:
                """

        # Choosing between compass orientations, defining the objectives and other variables
        current = self.corrCompass()
        if current == 0:
            if self.counter == 0:
                xin = self.round_even(self.measures.x)
                self.obj = xin + 2
                self.lin = 0.14
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.x
        elif current == 90:
            if self.counter == 0:
                yin = self.round_even(self.measures.y)
                self.obj = yin + 2
                self.lin = 0.14
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.y
        elif current == 180:
            if self.counter == 0:
                xin = self.round_even(self.measures.x)
                self.obj = xin - 2
                self.lin = 0.14
                self.integral = 0
                self.minus = True
            err = -self.obj + self.measures.x
        elif current == -90:
            if self.counter == 0:
                yin = self.round_even(self.measures.y)
                self.obj = yin - 2
                self.lin = 0.14
                self.integral = 0
                self.minus = True
            err = -self.obj + self.measures.y
        else:
            err = 0

        # Calculates the velocity based on the PID
        if self.lin != 0:
            diff = err / self.lin
        else:
            diff = 100
        self.integral += err
        self.lin = Kp * err + Kd * diff + Ki * self.integral
        self.length = err

        # PID controller to keep the robot moving in a straight line
        objective = current
        self.rotate(1, 0, 0, objective, True)

        # Limiting the velocities
        if self.lin > 0.14:
            self.lin = 0.14
        if self.rot > 0.15:
            self.rot = 0.15

        # Sending the velocities
        self.converter(self.lin, self.rot)
        self.counter += 1

        # If the length to be walked is under a certain threshold
        if -0.11 < self.length < 0.11:

            # Check if it is in a beacon, and if so, annotates it
            if self.measures.ground != -1 :
                if (self.round_even(self.measures.x), self.round_even(self.measures.y)) not in self.beacon_coordinates:
                    self.beacon_coordinates.append((self.round_even(self.measures.x), self.round_even(self.measures.y)))
                    self.beacon_nums.append(self.measures.ground)
            # Defines a new objective
            # if self.minus:
            #     self.obj -= 2
            # else:
            #     self.obj += 2
            self.counter = 0

            # Placing the current and previous location on the map matrix. Calls the wall function to map the walls.
            if current == 0:
                x = self.round_even(self.measures.x)
                self.walls(0, self.last_x + 2, self.last_y)
                self.maze.matrix[self.last_y][self.last_x + 1] = 'X'
                self.maze.matrix[self.last_y][self.last_x + 2] = 'X'
                self.last_x = x + 27
            elif current == 90:
                y = self.round_even(self.measures.y)
                self.walls(90, self.last_x, self.last_y - 2)
                self.maze.matrix[self.last_y - 1][self.last_x] = 'X'
                self.maze.matrix[self.last_y - 2][self.last_x] = 'X'
                self.last_y = -y + 13
            elif current == 180:
                x = self.round_even(self.measures.x)
                self.walls(180, self.last_x - 2, self.last_y)
                self.maze.matrix[self.last_y][self.last_x - 1] = 'X'
                self.maze.matrix[self.last_y][self.last_x - 2] = 'X'
                self.last_x = x + 27
            elif current == -90:
                y = self.round_even(self.measures.y)
                self.walls(-90, self.last_x, self.last_y + 2)
                self.maze.matrix[self.last_y + 1][self.last_x] = 'X'
                self.maze.matrix[self.last_y + 2][self.last_x] = 'X'
                self.last_y = -y + 13


            # If the movement is over, return true. If not, return false
            return True
        return False

    def walls(self, compass, x, y):
        """
                From the GPS location, the compass orientation and the values of the proximity sensore, determine where are
                walls and maps them.
                :param compass:
                :param x:
                :param y:
                :return:
                """
        str = None
        value_to_detect = 1.2

        if compass == 0:
            if self.measures.irSensor[0] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[
                2] >= value_to_detect:
                str = 'deadend 13'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 6'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 8'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[2] >= value_to_detect:
                str = 'both walls'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= value_to_detect:
                str = 'right wall'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[1] >= value_to_detect:
                str = 'left wall'
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[0] >= value_to_detect:
                str = 'wall in front'
                self.maze.matrix[y][x + 1] = '|'

        elif compass == 90:
            if self.measures.irSensor[0] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[
                2] >= value_to_detect:
                str = 'deadend 14'
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 5'
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 6'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect:
                str = 'both walls'
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect:
                str = 'left wall'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[2] >= value_to_detect:
                str = 'right wall'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[0] >= value_to_detect:
                str = 'wall in front'
                self.maze.matrix[y - 1][x] = '-'

        elif compass == 180:
            if self.measures.irSensor[0] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[
                2] >= value_to_detect:
                str = 'deadend 15'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 7'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 5'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[2] >= value_to_detect:
                str = 'both walls'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[1] >= value_to_detect:
                str = 'left wall'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= value_to_detect:
                str = 'right wall'
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[0] >= value_to_detect:
                str = 'wall in front'
                self.maze.matrix[y][x - 1] = '|'

        elif compass == -90:
            if self.measures.irSensor[0] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[
                2] >= value_to_detect:
                str = 'deadend 12'
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 8'
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[0] >= value_to_detect:
                str = 'corner 7'
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= value_to_detect and self.measures.irSensor[1] >= value_to_detect:
                str = 'both walls'
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= value_to_detect:
                str = 'left wall'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= value_to_detect:
                str = 'right wall'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[0] >= value_to_detect:
                str = 'wall in front'
                self.maze.matrix[y + 1][x] = '-'

        if str:
            logging.info(f'Mapped as {str}')


    def rotate(self, Kp, Kd, Ki, obj, retrot):
        """
        PID to rotate
        :param Kp: Proportional constant for PID
        :param Kd: Directional constant for PID
        :param Ki: Integral constant for PID
        :param obj: Rotation objective
        :param retrot: If it's rotating fully or only correcting the direction when walking in front
        :return:
        """

        # If it's rotating for the first time, define variables
        if self.counter2 == 0:
            self.rot = 0.15
            self.integralrot = 0

        # Calculate the error
        err = (obj - self.measures.compass) * math.pi / 180

        # Using a PID to define angular velocity
        if self.rot != 0:
            diff = err / self.rot
        else:
            diff = 100
        self.integralrot += err
        self.rot = Kp * err + Kd * diff + Ki * self.integralrot
        self.lengthrot = err

        # If the rotation is fully, send the values to the converter
        if not retrot:
            self.converter(0, self.rot)
            self.counter = 0
        self.counter2 += 1

        # If the rotation to rotated is under a certain threshold, stop the rotation, If not, continue
        if -0.005 < self.lengthrot < 0.005:
            self.counter2 = 0
            return False
        return True

    def corrCompass(self):
        """
        Corrects the compass position to the nearest cardinal point
        :return:
        """

        current = self.measures.compass
        if -45 < current < 45:
            current = 0
        elif 45 < current < 135:
            current = 90
        elif 135 < current or current < -135:
            current = 180
        elif -100 < current < -80:
            current = -90

        return current

    def compare_compass(self):
        current = self.measures.compass
        objective = self.corrCompass()
        difference = abs(objective - current)
        return difference

    def whosFree(self):
        """
        See which direction has a wall
        """

        current = self.corrCompass()

        if self.measures.irSensor[1] < 1:
            self.objective = current + 90
        elif self.measures.irSensor[2] < 1:
            self.objective = current - 90
        # elif self.measures.irSensor[3] < 1:
        #     self.objective = current + 180
        else:
            self.objective = current + 180
            # print('''I'm lost, please help me''')

        if self.objective <= -180:
            self.objective += 360
        if self.objective > 180:
            self.objective -= 360
        logging.info(f'Found an empty spot at {self.objective}')

    def gpsConverter(self):
        """
        Convert gps coordinates from absolute to relative
        :return:
        """

        if self.countergps == 0:
            self.xin = self.measures.x
            self.yin = self.measures.y
            self.countergps += 1
        self.measures.x -= self.xin
        self.measures.y -= self.yin

    def checkChangeCompass(self):
        """
        If the robot is in any way facing south, toggle a variable
        :return:
        """
        if self.objective == 180:
            logging.debug('Facing South')
            self.South = True
        else:
            self.South = False

    def appendWalked(self):
        """
        Append every coordinate walked to a list
        :return:
        """
        # Get GPS values
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        self.walked.append((x, y))

    def searchUnknown(self):
        """
        Search in all 4 directions for empty spaces and places them on a list
        :return:
        """
        # Get GPS and compass values
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        current = radians(self.corrCompass())
        entries = []

        # If a surrounding cell is empty, add it to the list
        if self.measures.irSensor[0] < 1:
            entries.append((x + round(cos(current)), y + round(sin(current))))
        if self.measures.irSensor[1] < 1:
            entries.append((x + round(cos(current + pi / 2)), y + round(sin(current + pi / 2))))
        # if self.measures.irSensor[3] < 1:
        #     entries.append((x + round(cos(current + pi)), y + round(sin(current + pi))))
        if self.measures.irSensor[2] < 1:
            entries.append((x + round(cos(current - pi / 2)), y + round(sin(current - pi / 2))))

        # Avoid repetition between lists
        for entry in entries:
            if entry not in self.unknown and entry not in self.known:
                logging.info(f'Added {entry} to unknown list')
                self.unknown.append(entry)

    def searchKnown(self):
        """
        When the robot is in a cell, it's certain that cell is empty. Append it to a list.
        :return:
        """
        # Get GPS values
        x = self.round_even(self.measures.x)
        y = self.round_even(self.measures.y)
        entry = (x, y)
        last_entry = self.walked[-2]
        mid_entry = (int((last_entry[0] + entry[0]) / 2), (int((last_entry[1] + entry[1]) / 2)))
        # If the first wall is covered
        equal = last_entry == mid_entry
        if equal:
            # self.first = False
            logging.debug('My last coord is the same as the current one.')

        # Append the coordinates if they are not there already, and remove if on unknown
        if entry in self.unknown:
            self.unknown.remove(entry)
            logging.info(f'Removed {entry} of unknown list')
        # if mid_entry in self.unknown and not self.first and not equal:
        if mid_entry in self.unknown and not self.first and not equal:
            self.unknown.remove(mid_entry)
            logging.info(f'Removed {mid_entry} of unknown list')
        # if mid_entry not in self.known and not self.first and not equal:
        if mid_entry not in self.known and not equal:
            self.known.append(mid_entry)
            logging.info(f'Added {mid_entry} to known list')
        # elif mid_entry not in self.known and self.first and not equal:
        #     self.unknown.append(mid_entry)
        #     self.first = False
        #     logging.info(f'Added {mid_entry} to unknown list')
        if entry not in self.known:
            self.known.append(entry)
            logging.info(f'Added {entry} to known list')
            return False
        else:
            return True

    def velEstimator(self, left_motor, right_motor):
        """
        Estimates the real velocity from the commands given
        """
        left_out = (left_motor + self.estimated_velocity[-1][0]) / 2
        right_out = (right_motor + self.estimated_velocity[-1][1]) / 2
        self.estimated_velocity.append((left_out, right_out))
        self.kinematics()

    def kinematics(self):
        """
        From each wheel velocity, calculate the velocity in the global coordinate frame
        """
        wheel_velocity = np.transpose(np.asarray(self.estimated_velocity[-1]))
        velocity_matrix = np.asarray([[1 / 2, 1 / 2], [-1 / 2, 1 / 2]])
        global_velocity_matrix = np.asarray([[math.cos(math.radians(self.measures.compass)), 0],
                                             [math.sin(math.radians(self.measures.compass)), 0],
                                             [0, 1]])
        velocity = np.matmul(velocity_matrix, wheel_velocity)
        global_velocity = np.matmul(global_velocity_matrix, velocity)
        current_velocity = (global_velocity[0], global_velocity[1])
        last_pose = self.pose[-1]
        current_pose = (last_pose[0] + current_velocity[0], last_pose[1] + current_velocity[1])
        corrected_pose = self.corrector(last_pose)
        if corrected_pose:
            current_pose = (corrected_pose[0] + current_velocity[0], corrected_pose[1] + current_velocity[1])
        self.pose.append(current_pose)

    def distance(self, x):
        return 1 / x
    
    def corrector(self, last_pose):
        current_pose = None
        wall = None
        direction = ""
        old_pose = last_pose
        center = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        robot_radius = 0.5 #0.5 diameter
        distance_to_wall = 0.9
        difference_threshold = 2
        value_to_front = 1.2
        value_to_min_side = 0.4
        value_to_max_side = 2.0
        distance_threshold = 2.0
        # logging.debug(f'Left: {self.side_correction}')

        if self.compare_compass() <= difference_threshold:
            if self.corrCompass() == 0:
                if center >= value_to_front and back >= value_to_front:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance((center + back)/2) - robot_radius, last_pose[1])
                    last_pose = current_pose
                    direction = 'Front '
                # elif (self.left_detected and left <= value_to_min_side) or (self.right_detected and right <= value_to_min_side):
                #     current_pose = (self.round_odd(last_pose[0]) + 0.35, last_pose[1])
                #     last_pose = current_pose
                #     self.left_detected = False
                #     self.right_detected = False
                #     direction = 'Sides '
                if left >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance(left) - robot_radius)
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance(right) + robot_radius)
                    direction = direction + 'Right'

            elif self.corrCompass() == 90:
                if center >= value_to_front and back >= value_to_front:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance((center + back)/2) - robot_radius)
                    last_pose = current_pose
                    direction = 'Front'
                # elif (self.left_detected and left <= value_to_min_side) or (self.right_detected and right <= value_to_min_side):
                #     current_pose = (last_pose[0], self.round_odd(last_pose[1]) + 0.35)
                #     last_pose = current_pose
                #     self.left_detected = False
                #     self.right_detected = False
                #     direction = 'Sides '
                if left >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance(left) + robot_radius, last_pose[1])
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance(right) - robot_radius, last_pose[1])
                    direction = direction + 'Right'

            elif self.corrCompass() == 180:
                if center >= value_to_front and back >= value_to_front:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance((center + back)/2) + robot_radius, last_pose[1])
                    last_pose = current_pose
                    direction = 'Front'
                # elif (self.left_detected and left <= value_to_min_side) or (self.right_detected and right <= value_to_min_side):
                #     current_pose = (self.round_odd(last_pose[0]) - 0.35, last_pose[1])
                #     last_pose = current_pose
                #     self.left_detected = False
                #     self.right_detected = False
                #     direction = 'Sides '
                if left >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance(left) + robot_radius)
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = last_pose[0], self.round_even(last_pose[1]) + distance_to_wall
                    current_pose = (last_pose[0], wall[1] - self.distance(right) - robot_radius)
                    direction = direction + 'Right'

            elif self.corrCompass() == -90:
                if center >= value_to_front and back >= value_to_front:
                    wall = last_pose[0], self.round_even(last_pose[1]) - distance_to_wall
                    current_pose = (last_pose[0], wall[1] + self.distance((center + back)/2) + robot_radius)
                    last_pose = current_pose
                    direction = 'Front'
                # elif (self.left_detected and left <= value_to_min_side) or (self.right_detected and right <= value_to_min_side):
                #     current_pose = (last_pose[0], self.round_odd(last_pose[1]) - 0.35)
                #     last_pose = current_pose
                #     self.left_detected = False
                #     self.right_detected = False
                #     direction = 'Sides '
                if left >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) + distance_to_wall, last_pose[1]
                    current_pose = (wall[0] - self.distance(left) - robot_radius, last_pose[1])
                    direction = direction + 'Left'
                elif right >= value_to_max_side:
                    wall = self.round_even(last_pose[0]) - distance_to_wall, last_pose[1]
                    current_pose = (wall[0] + self.distance(right) + robot_radius, last_pose[1])
                    direction = direction + 'Right'

            if current_pose:
                current_pose = (round(current_pose[0], 3), round(current_pose[1], 3))
                old_pose = (round(old_pose[0], 3), round(old_pose[1], 3))
                logging.debug(f'Wall close to the {direction}, I believe I am at {current_pose} and I believed I was at {old_pose}')
                if wall:
                    logging.debug(f'Wall coordinates: {wall}')
                return current_pose

    def round_even(self, number):
        return round(number/2)*2

    def round_odd(self, number):
        difference = number - self.round_even(number)
        if difference >= 0:
            return round(number/2)*2 + 1
        else:
            return round(number/2)*2 - 1


    def converter(self, lin, rot):
        """
        Converts the value of linear and angular velocity in motor rotation
        :param lin: Float32
        :param rot: Float32
        :return:
        """
        left_motor = lin - rot / 2
        right_motor = lin + rot / 2
        if left_motor > 0.15:
            left_motor = 0.15
        elif left_motor < -0.15:
            left_motor = -0.15
        if right_motor > 0.15:
            right_motor = 0.15
        elif right_motor < -0.15:
            right_motor = -0.15
        logging.debug(f'The velocity command given to the converter is (lin: {round(lin, 3)}, rot: {round(rot, 3)})')
        logging.debug(f'The velocity command given to the motors is ({round(left_motor, 3)},{round(right_motor, 3)})')
        if not self.onRot:
            self.velEstimator(left_motor, right_motor)
        self.driveMotors(left_motor, right_motor)


class Lab():
    def __init__(self):
        self.matrix = [[' '] * 55]

        for m in range(26):
            self.matrix.insert(0, [' '] * 55)
        self.matrix[13][27] = 'I'


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "veryimportantrobot"
host = "localhost"
pos = 1
mapc = None
f = "default"

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = sys.argv[i + 1]
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        f = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    logging.basicConfig(filename='app.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    # set up logging to console
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(asctime)s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    logger = logging.getLogger(__name__)
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 0.0], host)
    rob.f = f
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
