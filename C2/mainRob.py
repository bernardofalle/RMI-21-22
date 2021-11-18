#!/usr/bin/python3
import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from time import sleep
from astar import *


CELLROWS=7
CELLCOLS=14

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
        self.maze=Lab()
        self.last_x = 27
        self.last_y = 13
        self.unknown = []
        self.known = [(0, 0)]
        self.path=[]
        self.searching=False
        self.target=None,None

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
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        maze=Lab()
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        center_sensor = self.measures.irSensor[center_id]

        # Offset the gps, making the coordinates relative to the starting point
        self.gpsConverter()

        # Check if the compass is facing south
        self.checkChangeCompass()

        # If you are facing south, offset the compass
        if self.South and self.measures.compass < -90:
            self.measures.compass += 360

        # If you have travelled a distance of 2
        if self.endCycle:
            # Check if there is a wall in front of you or you are rotating
            self.searching=False
            if center_sensor > 1.2 or self.onRot:
                # If it's the first time it is running this cycle, check the surrounding
                # environment for a free space to go to
                #print(self.counterfree)
                #print(self.searching)
                if self.counterfree == 0:
                    if self.searching:
                        self.searching = False
                    else:
                        print('werq')
                        self.whosFree()
                    self.counterfree += 1

                # Start rotating to the available free space. Once it is done, this function returns false
                self.onRot = self.rotate(3, 0, 0, self.objective, False)
            elif self.searching:
                    print('Searching')

                    self.path = self.path[1:]
                    print('Path: ' + str(self.path))
                    loc = round(self.measures.x), round(self.measures.y)
                    x, y = (self.path[0][0] - loc[0]), (self.path[0][1] - loc[1])
                    print('Loc: ' + str(loc))
                    #print('X: ' + str(x) + ' Y: ' + str(y))
                    self.onRot = True
                    if x < 0:
                        self.objective = 180
                        print('rotating 180')
                    elif x > 0:
                        self.objective = 0
                        print('coming through')
                    elif y < 0:
                        self.objective = -90
                        print('rotating -90')
                    elif y > 0:
                        self.objective = 90
                        print('rotating 90')
                    else:
                        print('Test')
                        self.onRot = False



                    if len(self.path) == 1:
                        self.path = []
                        self.counterfree=0
                        #self.known.append(self.unknown[0])
                        #self.unknown=self.unknown[1:]
                        self.searching=False
            else:
                # If it doesn't have anything in front nor is in middle of a rotation, start a new cycle,
                # restart variables and annotate known and unknown variables
                
                self.amknown = self.searchKnown()
                me=round(self.measures.x),round(self.measures.y)
                print(self.known)
                self.searchUnknown()
                self.endCycle = False
                self.counterfree = 0
                print(self.amknown)
                if self.South:
                    self.South = False
                if self.amknown:
                    start=round(self.measures.x),round(self.measures.y)
                    self.a(start, self.unknown[0])
                    self.path.append(self.unknown[0])
                    print('debug')
                    self.searching = True
                else:
                    self.searching=False
        else:
            # If you are not in the end of a cycle, move in front
            self.endCycle = self.moveFront(0.1, 0.01, 0.00005)
        #print(self.known)
        #print(self.unknown)
        
        self.writeMap()



    def writeMap(self):
        f=open('mapping.out','w+')
        for line in self.maze.matrix:
            for element in line:
                f.write(element)
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

        # Choosing between situations
        current = self.corrCompass()
        if current == 0:
            if self.counter == 0:
                xin = round(self.measures.x)
                self.obj = xin + 2
                self.lin = 0.15
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.x
        elif current == 90:
            if self.counter == 0:
                yin = round(self.measures.y)
                self.obj = yin + 2
                self.lin = 0.15
                self.integral = 0
                self.minus = False
            err = self.obj - self.measures.y
        elif current == 180:
            if self.counter == 0:
                xin = round(self.measures.x)
                self.obj = xin - 2
                self.lin = 0.15
                self.integral = 0
                self.minus = True
            err = -self.obj + self.measures.x
        elif current == -90:
            if self.counter == 0:
                yin = round(self.measures.y)
                self.obj = yin - 2
                self.lin = 0.15
                self.integral = 0
                self.minus = True
            err = -self.obj + self.measures.y

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
        if self.rot > 0.14:
            self.rot = 0.14

        self.converter(self.lin, self.rot)
        self.counter += 1


        if -0.11 < self.length < 0.11:
            if self.minus:
                self.obj -= 2
            else:
                self.obj += 2

            print('Mapping...')

            # print(self.path)
            
            #if start in self.known and self.searching is False:
            #    self.a(start,self.unknown[0])
            #    self.searching=True

            if current==0:
                x=round(self.measures.x)
                self.walls(0,self.last_x+2,self.last_y)
                self.maze.matrix[self.last_y][self.last_x+1]='X'
                self.maze.matrix[self.last_y][self.last_x+2]='X'
                self.last_x=x+27
            elif current==90:
                y=round(self.measures.y)
                self.walls(90,self.last_x,self.last_y-2)
                self.maze.matrix[self.last_y-1][self.last_x]='X'
                self.maze.matrix[self.last_y-2][self.last_x]='X'
                self.last_y=-y+13
            elif current==180:
                x=round(self.measures.x)
                self.walls(180,self.last_x-2,self.last_y)
                self.maze.matrix[self.last_y][self.last_x-1]='X'
                self.maze.matrix[self.last_y][self.last_x-2]='X'
                self.last_x=x+27
            elif current==-90:
                y=round(self.measures.y)
                self.walls(-90,self.last_x,self.last_y+2)
                self.maze.matrix[self.last_y+1][self.last_x]='X'
                self.maze.matrix[self.last_y+2][self.last_x]='X'
                self.last_y=-y+13

            return True
        return False

    def walls(self,compass,x,y):
        
        if compass==0:
            if self.measures.irSensor[1]>=1.7 and self.measures.irSensor[0]>=1.5:
                print('corner 6')
                self.maze.matrix[y-1][x]='-'
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 8')
                self.maze.matrix[y+1][x]='-'
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[1]>=1.7 and self.measures.irSensor[2]>=1.7:
                print('both walls')
                self.maze.matrix[y-1][x]='-'
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[2]>=1.7:
                print('right wall')
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[1]>=1.7:
                print('left wall')
                self.maze.matrix[y-1][x]='-'
            elif self.measures.irSensor[0]>=1.7:
                print('wall in front')
                self.maze.matrix[y][x+1]='|'
            
        elif compass==90:
            if self.measures.irSensor[1]>=1.7 and self.measures.irSensor[0]>=1.5:
                print('corner 5')
                self.maze.matrix[y][x-1]='|'
                self.maze.matrix[y-1][x]='-'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 6')
                self.maze.matrix[y-1][x]='-'
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[1]>=1.7:
                print('both walls')
                self.maze.matrix[y][x-1]='|'
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[1]>=1.7:
                print('left wall')
                self.maze.matrix[y][x-1]='|'
            elif self.measures.irSensor[2]>=1.7:
                print('right wall')
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[0]>=1.7:
                print('wall in front')
                self.maze.matrix[y-1][x]='-'
            
        elif compass==180:
            if self.measures.irSensor[1]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 7')
                self.maze.matrix[y+1][x]='-'
                self.maze.matrix[y][x-1]='|'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 5')
                self.maze.matrix[y-1][x]='-'
                self.maze.matrix[y][x-1]='|'
            elif self.measures.irSensor[1]>=1.7 and self.measures.irSensor[2]>=1.7:
                print('both walls')
                self.maze.matrix[y-1][x]='-'
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[1]>=1.7:
                print('left wall')
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[2]>=1.7:
                print('right wall')
                self.maze.matrix[y-1][x]='-'
            elif self.measures.irSensor[0]>=1.7:
                print('wall in front')
                self.maze.matrix[y][x-1]='|'
            
        elif compass==-90:
            if self.measures.irSensor[1]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 8')
                self.maze.matrix[y][x+1]='|'
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[0]>=1.7:
                print('corner 7')
                self.maze.matrix[y][x-1]='|'
                self.maze.matrix[y+1][x]='-'
            elif self.measures.irSensor[2]>=1.7 and self.measures.irSensor[1]>=1.7:
                print('both walls')
                self.maze.matrix[y][x-1]='|'
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[1]>=1.5:
                print('left wall')
                self.maze.matrix[y][x+1]='|'
            elif self.measures.irSensor[2]>=1.7:
                print('right wall')
                self.maze.matrix[y][x-1]='|'
            elif self.measures.irSensor[0]>=1.7:
                print('wall in front')
                self.maze.matrix[y+1][x]='-'

    def rotate(self, Kp, Kd, Ki, obj, retrot):
        """
        PID to rotate
        :param Kp:
        :param Kd:
        :param Ki:
        :param obj:
        :param retrot:
        :return:
        """

        if self.counter2 == 0:
            self.rot = 0.15
            self.integralrot = 0

        err = (obj - self.measures.compass) * math.pi / 180

        # print(err)
        if self.rot != 0:
            diff = err / self.rot
        else:
            diff = 100
        self.integralrot += err
        self.rot = Kp * err + Kd * diff + Ki * self.integralrot
        self.lengthrot = err
        if not retrot:
            self.converter(0, self.rot)
            self.counter = 0
        self.counter2 += 1


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

    def a(self,start,goal):
        neighbours=[(0,2),(0,-2),(2,0),(-2,0)]

        for i,j in neighbours:
            neigh=goal[0]+i,goal[1]+j
            if neigh in self.known:
                 self.path=astar(self.known,start,neigh)

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
            print(self.objective)
        else:
            print('''I'm lost, please help me''')

        if self.objective <= -180:
            self.objective += 360
        if self.objective >=360:
            self.objective -=360

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
            entries.append((x + round(2*cos(current)), y + round(2*sin(current))))
        if self.measures.irSensor[1] < 1:
            entries.append((x + round(2*cos(current + pi/2)), y + round(2*sin(current + pi/2))))
        if self.measures.irSensor[3] < 1:
            entries.append((x + round(2*cos(current + pi)), y + round(2*sin(current + pi))))
        if self.measures.irSensor[2] < 1:
            entries.append((x + round(2*cos(current - pi/2)), y + round(2*sin(current - pi/2))))

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
        print(entry)
        # Append the coordinates if they are not there already, and remove if on unknown
        if entry in self.unknown:
            self.unknown.remove(entry)
        if entry not in self.known:
            print('entersdfafasf')
            self.known.append(entry)
            return False
        else:
            return True



    def converter(self, lin, rot):
        """
        Converts the value of linear and angular velocity in motor rotation
        :param lin: Float32
        :param rot: Float32
        :return:
        """
        left_motor = lin - rot / 2
        right_motor = lin + rot / 2
        self.driveMotors(left_motor, right_motor)

class Lab():
    def __init__(self):
        self.matrix=[[' ']*55]

        for m in range(26):
            self.matrix.insert(0,[' ']*55)
        self.matrix[13][27]='I'
    
class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "veryimportantrobot"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
