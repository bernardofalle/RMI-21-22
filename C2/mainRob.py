#!/usr/bin/python3
import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from time import sleep


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
        self.endCycle = False
        self.onRot = False

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
                    self.setVisitingLed(True);
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

        self.gpsConverter()
        print(self.measures.compass)
        if self.endCycle:
            if center_sensor > 1.2 or self.onRot:
                if self.counterfree == 0:
                    self.whosFree()
                    print('I have a wall in front of me, rotating to ' + str(self.objective) + 'º')
                    self.counterfree += 1
                self.onRot = self.rotate(3, 0, 0, self.objective, False)
            else:
                print('Open field, coming through!')
                self.endCycle = False
                self.counterfree = 0
        else:
            self.endCycle = self.moveFront(50, 0.01, 0.00005)





    def writeMap(self):
        f=open('mapping.txt','w')

        f.close()

    def moveFront(self, Kp, Kd, Ki):
        """
        PID for moving in front
        :param Kp:
        :param Kd:
        :param Ki:
        :return:
        """
        current = self.corrCompass()
        if current == 0:
            if self.counter == 0:
                xin = self.measures.x
                self.obj = xin + 2
                self.lin = 0.15
                self.integral = 0
            err = self.obj - self.measures.x
        elif current == 90:
            if self.counter == 0:
                yin = self.measures.y
                self.obj = yin + 2
                self.lin = 0.15
                self.integral = 0
            err = self.obj - self.measures.y
            print('Err: ' + str(err))
        elif current == 180:
            if self.counter == 0:
                xin = self.measures.x
                self.obj = xin - 2
                self.lin = 0.15
                self.integral = 0
            err = -self.obj + self.measures.x
        elif current == -90:
            if self.counter == 0:
                yin = self.measures.y
                self.obj = yin - 2
                self.lin = 0.15
                self.integral = 0
            err = -self.obj + self.measures.y


        if self.lin != 0:
            diff = err / self.lin
        else:
            diff = 100
        self.integral += err
        self.lin = Kp * err + Kd * diff + Ki * self.integral
        self.length = err

        objective = current
        self.rotate(15, 0, 0.1, objective, True)

        # print('Going with ' + str(self.lin) + ' linear and ' + str(self.rot) + ' angular velocity.')
        self.converter(self.lin, self.rot)
        self.counter += 1

        if -0.11 < self.length < 0.11:
            self.obj += 2
            print('Mapping...')

            return True
        return False

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
            self.integral = 0

        err = (obj - self.measures.compass) * math.pi / 180

        # print(err)
        if self.rot != 0:
            diff = err / self.rot
        else:
            diff = 100
        self.integral += err
        self.rot = Kp * err + Kd * diff + Ki * self.integral
        self.lengthrot = err
        if not retrot:
            self.converter(0, self.rot)
            self.counter = 0
        else:
            a = 1
            # print('Integral: ' + str(self.integral) + '\n Diff: ' + str(diff) + '\n Err: ' + str(err))
        self.counter2 += 1


        if -0.005 < self.lengthrot < 0.005:
            self.counter2 = 0
            print('Turned to ' + str(obj) + 'º')
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


    def converter(self, lin, rot):
        left_motor = lin - rot / 2
        right_motor = lin + rot / 2
        self.driveMotors(left_motor, right_motor)

class Lab():
    def __init__(self):
        self.colunms=[' ']*(CELLCOLS*4-1)
        self.rows=[' ']*(CELLROWS*4-1)
    
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
