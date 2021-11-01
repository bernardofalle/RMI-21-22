#!/usr/bin/python3

import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET



CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.posList = []
        self.errList = []

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
        lin = 0.15
        rot = 0
        center_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]
        back_sensor = self.measures.irSensor[back_id]
        ground_sensor = self.measures.ground
        compass = self.measures.compass
        x = self.measures.x
        y = self.measures.y

        self.moveFront(2, 0.2, 3, 0.001)

        # if counter >= 2:
        #     linVel = self.getLinVel(x, 1)
        #     # self.errorCalculator(lvSet, linVel)
        #     # lin = self.PIDControler(linVel, 0, 0, 0)
        #     lin = 0.15
        #     rot = 0
        #     print(linVel)
        # else:
        #     lin = 0.15
        #     rot = 0
        #     self.errorCalculator(lvSet, lin)
        #
        # self.converter(lin, rot)
        # counter += 1
        # if center_sensor < 5.0 and center_sensor > 0.2:
        #     lin = 0.15 / center_sensor
        # elif center_sensor < 0.2:
        #     lin = 0.15
        # else:
        #     lin = 0

        

    def writeMap(self):
        f=open('mapping.txt','w')

        f.close()


    def getLinVel(self, list, timeInt):
        currPos = self.posList[-1]
        prevPos = self.posList[-2]

        linVel = (currPos - prevPos) / timeInt

        return linVel

    def getAngVel(self, currAng, timeInt):
        global prevAng

        if prevAng is None:
            prevAng = 0

        angVel = currAng - prevAng / timeInt
        prevAng = currAng

        return angVel

    def errorCalculator(self, setpoint, current):
        error = setpoint - current
        self.errList.append(error)
        return

    def PIDControler(self,  prev, K0, K1, K2):
        error_3 = self.errList[-3]
        error_2 = self.errList[-2]
        error_1 = self.errList[-1]

        vel = prev + K0 * error_1 + K1 * error_2 + K2 * error_3
        print('sent vel: ' + str(vel))

        return vel

    def moveFront(self, length, Kp, Kd, Ki):
        xin = self.measures.x
        xobj = xin + length
        lin = 0.15
        integral = 0
        print('test')
        while length > 0:
            x = self.measures.x
            err = xobj - x
            print(err)
            diff = err / lin
            integral += err
            lin = Kp * err + Kd * diff + Ki * integral
            print(lin)
            length -= err
            print('length: ' + str(length))
            self.converter(lin, 0)




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
