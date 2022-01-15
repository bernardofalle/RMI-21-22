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
        self.endCycle = False
        self.onRot = False
        self.minus = False
        self.South = False
        self.maze = Lab()
        self.last_x = 27
        self.last_y = 13
        self.unknown = []
        self.known = [(0, 0)]
        self.path = []
        self.searching = False
        self.walked = [(0, 0)]
        self.pathfollowing = False
        self.haspath = False
        self.beacon_coordinates = [(0, 0)]
        self.go_to_beacons = False
        self.f = None
        self.final_path = []
        self.estimated_velocity = [(0, 0)]
        self.pose = [(0, 0)]
        self.WallClose = False

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
        threshold_warn = 0.3
        difference_x = round(self.measures.x - self.real_x, 3)
        difference_y = round(self.measures.y - self.real_y, 3)
        logging.debug(f'The real GPS values are (X,Y): ({round(self.real_x, 3)},{round(self.real_y, 3)})')
        logging.debug(f'The calculates GPS values are: ({round(self.measures.x, 3)},{round(self.measures.y, 3)})')
        logging.debug(f'The difference between values is ({difference_x},{difference_y})')
        if difference_y >= threshold_warn or difference_x >= threshold_warn:
            logging.warning('A lot or error experienced')
        center_id = 0
        center_sensor = self.measures.irSensor[center_id]

        # Check if the compass is facing south
        self.checkChangeCompass()

        # If it is facing south, offset the compass
        if self.South and self.measures.compass < -90:
            self.measures.compass += 360

        # If it has travelled a distance of 2
        if self.endCycle:
            # If you are rotating
            if self.onRot:
                logging.debug('Rotating...')
                # Start rotating to the predefined. Once it is done, this function returns false
                self.onRot = self.rotate(3, 0, 0, self.objective, False)

            # If it is following a path and needs to locate the next position
            elif self.searching:
                # If the path has ended, reset the variables
                if len(self.path) == 0:
                    self.haspath = False
                    self.searching = False
                else:
                    # Find the current location of the robot
                    loc = round(self.measures.x), round(self.measures.y)

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
                    if self.objective != current:
                        # If the the objective orientation is different then the current one, rotate to it
                        self.onRot = True
                    else:
                        # If not, do not rotate, reset the searching variable (so it can move in front) and remove the
                        # next path coordinate
                        self.onRot = False
                        self.searching = False
                        self.path = self.path[1:]

            # If it is not following a path and there is an obstacle close to the front of the agent
            elif center_sensor > 1.2 and not self.pathfollowing:
                # Search its surroundings for an available path and rotates to it
                self.whosFree()
                self.onRot = True

            else:
                logging.debug(f'Ended cycle on ({self.measures.x},{self.measures.y})')
                # If it doesn't have anything in front, nor is in middle of a rotation, nor is it searching for a next
                # position, add the walked, known and unknown coordinates
                self.appendWalked()
                self.amknown = self.searchKnown()
                self.searchUnknown()

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
                        start = round(self.measures.x), round(self.measures.y)

                        # Define the list of possible ends
                        end_list = self.unknown

                        # From the list of possible ends and the current coordinates, search the smallest path
                        end = self.a(start, end_list)

                        # Remove the odd cells from the path, add the end and removing the start
                        self.path = [items for items in self.path if items[0] % 2 == 0 and items[1] % 2 == 0]
                        self.path.append((2 * end[0] - self.path[-1][0], 2 * end[1] - self.path[-1][1]))
                        self.path.remove(start)

                        # Start the variables
                        self.pathfollowing = True
                        self.haspath = True

                    else:
                        # If it has a path already, walk in front
                        self.endCycle = False

                # If it has not following paths, move in front
                if not self.pathfollowing:
                    self.endCycle = False

        else:
            # If it is not in the end of a cycle, move in front
            self.endCycle = self.moveFront(0.1, 0.01, 0.00005)

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
                    self.path, timeout = astar(self.maze.matrix, start, final_goal, time(), 0.5)

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
            self.beacon_coordinates.remove((0,0))
            perms = list(itertools.permutations(self.beacon_coordinates))
            path = []

            for perm in perms:
                perm.insert(0, (0,0))
                for i in range(0, len(perm)):
                    if i + 1 == len(perm):
                        p, timeout = astar(self.maze.matrix, perm[i], perm[0], time(), 0.5)
                        path.extend(p)
                    else:
                        p, timeout = astar(self.maze.matrix, perm[i], perm[i + 1], time(), 0.5)
                        path.extend(p)
                        path.remove(path[-1])
                
                if len(self.final_path == 0) :
                    self.final_path = path
                elif len(path) <= len(self.final_path) :
                    self.final_path = path
                path = []
            
            self.final_path = [i for i in self.final_path if i[0] % 2 == 0 and i[1] % 2 == 0]

            #start = self.beacon_coordinates[0]
            #goal = self.beacon_coordinates[1]
            #self.path, timeout = astar(self.maze.matrix, start, goal, time(), 0.5)
            # self.path.remove(start)
            #self.final_path.extend(self.path)
            #start = self.beacon_coordinates[1]
            #goal = self.beacon_coordinates[2]
            #self.path, timeout = astar(self.maze.matrix, start, goal, time(), 0.5)
            #self.path.remove(start)
            #self.final_path.extend(self.path)
            #start = self.beacon_coordinates[2]
            #goal = self.beacon_coordinates[0]
            #self.path, timeout = astar(self.maze.matrix, start, goal, time(), 0.5)
            #self.path.remove(start)
            #self.final_path.extend(self.path)
            #self.final_path = [i for i in self.final_path if i[0] % 2 == 0 and i[1] % 2 == 0]
            #print(self.final_path)

            self.writePath()
            #sys.exit()

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
                xin = round(self.measures.x)
                self.obj = xin + 2
                self.lin = 0.10
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.x
        elif current == 90:
            if self.counter == 0:
                yin = round(self.measures.y)
                self.obj = yin + 2
                self.lin = 0.10
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.y
        elif current == 180:
            if self.counter == 0:
                xin = round(self.measures.x)
                self.obj = xin - 2
                self.lin = 0.10
                self.integral = 0
                self.minus = True
            err = -self.obj + self.measures.x
        elif current == -90:
            if self.counter == 0:
                yin = round(self.measures.y)
                self.obj = yin - 2
                self.lin = 0.10
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
        if self.lin > 0.10:
            self.lin = 0.10
        if self.rot > 0.15:
            self.rot = 0.15

        # Sending the velocities
        self.converter(self.lin, self.rot)
        self.counter += 1

        # If the length to be walked is under a certain threshold
        if -0.11 < self.length < 0.11:

            # Check if it is in a beacon, and if so, annotates it
            if self.measures.ground == 1 or self.measures.ground == 2:
                if (round(self.measures.x), round(self.measures.y)) not in self.beacon_coordinates:
                    self.beacon_coordinates.append((round(self.measures.x), round(self.measures.y)))

            # Defines a new objective
            if self.minus:
                self.obj -= 2
            else:
                self.obj += 2

            # Placing the current and previous location on the map matrix. Calls the wall function to map the walls.
            if current == 0:
                x = round(self.measures.x)
                self.walls(0, self.last_x + 2, self.last_y)
                self.maze.matrix[self.last_y][self.last_x + 1] = 'X'
                self.maze.matrix[self.last_y][self.last_x + 2] = 'X'
                self.last_x = x + 27
            elif current == 90:
                y = round(self.measures.y)
                self.walls(90, self.last_x, self.last_y - 2)
                self.maze.matrix[self.last_y - 1][self.last_x] = 'X'
                self.maze.matrix[self.last_y - 2][self.last_x] = 'X'
                self.last_y = -y + 13
            elif current == 180:
                x = round(self.measures.x)
                self.walls(180, self.last_x - 2, self.last_y)
                self.maze.matrix[self.last_y][self.last_x - 1] = 'X'
                self.maze.matrix[self.last_y][self.last_x - 2] = 'X'
                self.last_x = x + 27
            elif current == -90:
                y = round(self.measures.y)
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
        if compass == 0:
            if self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[
                2] >= 1.5:
                print('deadend 13')
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 6')
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 8')
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] >= 1.5:
                print('both walls')
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= 1.5:
                print('right wall')
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[1] >= 1.5:
                print('left wall')
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[0] >= 1.5:
                print('wall in front')
                self.maze.matrix[y][x + 1] = '|'

        elif compass == 90:
            if self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[
                2] >= 1.5:
                print('deadend 14')
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 5')
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 6')
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[1] >= 1.5:
                print('both walls')
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= 1.5:
                print('left wall')
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[2] >= 1.5:
                print('right wall')
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[0] >= 1.5:
                print('wall in front')
                self.maze.matrix[y - 1][x] = '-'

        elif compass == 180:
            if self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[
                2] >= 1.5:
                print('deadend 15')
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 7')
                self.maze.matrix[y + 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 5')
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] >= 1.5:
                print('both walls')
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[1] >= 1.5:
                print('left wall')
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= 1.5:
                print('right wall')
                self.maze.matrix[y - 1][x] = '-'
            elif self.measures.irSensor[0] >= 1.5:
                print('wall in front')
                self.maze.matrix[y][x - 1] = '|'

        elif compass == -90:
            if self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[
                2] >= 1.5:
                print('deadend 12')
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y - 1][x] = '-'
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 8')
                self.maze.matrix[y][x + 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[0] >= 1.5:
                print('corner 7')
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y + 1][x] = '-'
            elif self.measures.irSensor[2] >= 1.5 and self.measures.irSensor[1] >= 1.5:
                print('both walls')
                self.maze.matrix[y][x - 1] = '|'
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[1] >= 1.5:
                print('left wall')
                self.maze.matrix[y][x + 1] = '|'
            elif self.measures.irSensor[2] >= 1.5:
                print('right wall')
                self.maze.matrix[y][x - 1] = '|'
            elif self.measures.irSensor[0] >= 1.5:
                print('wall in front')
                self.maze.matrix[y + 1][x] = '-'

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

    def whosFree(self):
        """
        See which direction has a wall
        """

        current = self.corrCompass()

        if self.measures.irSensor[1] < 1:
            self.objective = current + 90
        elif self.measures.irSensor[2] < 1:
            self.objective = current - 90
        elif self.measures.irSensor[3] < 1:
            self.objective = current + 180
        else:
            print('''I'm lost, please help me''')

        if self.objective <= -180:
            self.objective += 360
        if self.objective >= 360:
            self.objective -= 360

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
            self.South = True
        else:
            self.South = False

    def appendWalked(self):
        """
        Append every coordinate walked to a list
        :return:
        """
        # Get GPS values
        x = round(self.measures.x)
        y = round(self.measures.y)
        self.walked.append((x, y))

    def searchUnknown(self):
        """
        Search in all 4 directions for empty spaces and places them on a list
        :return:
        """
        # Get GPS and compass values
        x = round(self.measures.x)
        y = round(self.measures.y)
        current = radians(self.corrCompass())
        entries = []

        # If a surrounding cell is empty, add it to the list
        if self.measures.irSensor[0] < 1:
            entries.append((x + round(cos(current)), y + round(sin(current))))
        if self.measures.irSensor[1] < 1:
            entries.append((x + round(cos(current + pi / 2)), y + round(sin(current + pi / 2))))
        if self.measures.irSensor[3] < 1:
            entries.append((x + round(cos(current + pi)), y + round(sin(current + pi))))
        if self.measures.irSensor[2] < 1:
            entries.append((x + round(cos(current - pi / 2)), y + round(sin(current - pi / 2))))

        # Avoid repetition between lists
        for entry in entries:
            if entry not in self.unknown and entry not in self.known:
                self.unknown.append(entry)

    def searchKnown(self):
        """
        When the robot is in a cell, it's certain that cell is empty. Append it to a list.
        :return:
        """
        # Get GPS values
        x = round(self.measures.x)
        y = round(self.measures.y)
        entry = (x, y)
        last_entry = self.walked[-2]
        mid_entry = (int((last_entry[0] + entry[0]) / 2), (int((last_entry[1] + entry[1]) / 2)))
        # Append the coordinates if they are not there already, and remove if on unknown
        if entry in self.unknown:
            self.unknown.remove(entry)
        if mid_entry in self.unknown:
            self.unknown.remove(mid_entry)
        if mid_entry not in self.known:
            self.known.append(mid_entry)
        if entry not in self.known:
            self.known.append(entry)
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
        corrected_pose = self.corrector(current_pose)
        if corrected_pose:
            self.pose.append(corrected_pose)
        else:
            self.pose.append(current_pose)
    
    def distance(self, x):
        return 1 / x
    
    def corrector(self, last_pose):
        current_pose = None
        # last_pose = self.pose[-1]
        center = self.measures.irSensor[0]
        left = self.measures.irSensor[1]
        right = self.measures.irSensor[2]
        back = self.measures.irSensor[3]
        wall = 0,0
        robot_radius = 0.5 #0.5 diameter

        if self.corrCompass() == 0:
            if center >= 1.2:
                wall = self.round_even(last_pose[0]) + 0.8, last_pose[1]
                current_pose = (wall[0] - self.distance(center) - robot_radius, last_pose[1])
            # elif left >= 1.2 :
            #     wall = last_pose[0], last_pose[1] + 1
            #     current_pose = (last_pose[0], wall[1] - self.distance(left) - robot_radius)
            # elif right >= 1.2 :
            #     wall = last_pose[0], last_pose[1] - 1
            #     current_pose = (last_pose[0], wall[1] + self.distance(right) + robot_radius)

        elif self.corrCompass() == 90:
            if center >= 1.2:
                wall = last_pose[0], self.round_even(last_pose[1]) + 0.8
                current_pose = (last_pose[0], wall[1] - self.distance(center) - robot_radius)
            # elif left >= 1.2 :
            #     wall = last_pose[0] -  1, last_pose[1]
            #     current_pose = (wall[0] + self.distance(left) + robot_radius, last_pose[1])
            # elif right >= 1.2 :
            #     wall = last_pose[0] + 1, last_pose[1]
            #     current_pose = (wall[0] - self.distance(right) - robot_radius, last_pose[1])
        
        elif self.corrCompass() == 180:
            if center >= 1.2:
                wall = self.round_even(last_pose[0]) - 0.8, last_pose[1]
                current_pose = (wall[0] + self.distance(center) + robot_radius, last_pose[1])
            # elif left >= 1.2 :
            #     wall = last_pose[0], last_pose[1] - 1
            #     current_pose = (last_pose[0], wall[1] + self.distance(left) + robot_radius)
            # elif right >= 1.2 :
            #     wall = last_pose[0], last_pose[1] + 1
            #     current_pose = (last_pose[0], wall[1] - self.distance(right) - robot_radius)
        
        elif self.corrCompass() == -90:
            if center >= 1.2:
                wall = last_pose[0], self.round_even(last_pose[1]) - 0.8
                current_pose = (last_pose[0], wall[1] + self.distance(center) + robot_radius)
            # elif left >= 1.2 :
            #     wall = last_pose[0] +  1, last_pose[1]
            #     current_pose = (wall[0] - self.distance(left) - robot_radius, last_pose[1])
            # elif right >= 1.2 :
            #     wall = last_pose[0] - 1, last_pose[1]
            #     current_pose = (wall[0] + self.distance(right) + robot_radius, last_pose[1])
        
        if current_pose:
            print(f'Wall Close, I believe I am at {current_pose}')
            print(f'Wall Close, I believe I was at {last_pose}')
            return current_pose

    def round_even(self, number):
        return round(number/2)*2

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
        if right_motor > 0.15:
            right_motor = 0.15
        # logging.debug(f'The velocity command given to the motors is ({round(left_motor, 3)},{round(right_motor, 3)})')
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
f = 'pathC3.out'

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
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
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    logger = logging.getLogger(__name__)
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    rob.f = f
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
