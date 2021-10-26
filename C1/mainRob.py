#!/usr/bin/python3

import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14
global counter
global ground_sensor_list
global full_list
counter = 0
ground_sensor_list = []
full_list = []

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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
        global counter
        global ground_sensor_list
        global full_list
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        lin = 0.15
        rot = 0
        default_list = [0, 1, 2]
        counter_max = 15
        center_sensor = self.measures.irSensor[center_id]
        left_sensor = self.measures.irSensor[left_id]
        right_sensor = self.measures.irSensor[right_id]
        back_sensor = self.measures.irSensor[back_id]
        ground_sensor = self.measures.ground
        print(ground_sensor)
        ground_sensor_list, full_list = self.recorder(ground_sensor, ground_sensor_list, full_list)
        if len(ground_sensor_list) >= 2:
            right_dir = self.comparer(ground_sensor_list, full_list, default_list)
        else:
            right_dir = True

        if right_dir:
            if center_sensor < 5.0 and center_sensor > 0.2:
                lin = 0.15 / center_sensor
                print('Slowing down')
                if left_sensor > 2.17 and right_sensor < 2.17:
                    rot = -0.05 * left_sensor
                    print('Turning right')
                elif right_sensor > 2.17 and left_sensor < 2.17:
                    rot = 0.05 * right_sensor
                    print('Turning left')
                elif left_sensor < right_sensor:
                    rot = 0.02 * left_sensor
                    print('Turning slightly right')
                elif left_sensor >= right_sensor:
                    rot = -0.02 * right_sensor
                    print('Turning slightly left')
                else:
                    rot = 0
                    print('Not turning')
            elif center_sensor > 5.0:
                lin = 0
                print('Stopped')
                if left_sensor > 2.17 and right_sensor < 2.17:
                    rot = -0.05 * left_sensor
                    print('Turning right')
                elif right_sensor > 2.17 and left_sensor < 2.17:
                    rot = 0.05 * right_sensor
                    print('Turning left')
                elif left_sensor < right_sensor:
                    rot = 0.02 * left_sensor
                    print('Turning slightly right')
                elif left_sensor >= right_sensor:
                    rot = -0.02 * right_sensor
                    print('Turning slightly left')
                else:
                    rot = 0
                    print('Not turning')
            elif center_sensor < 0.2:
                print('Full Speed')
                lin = 0.15

        else:
            if counter <= counter_max:
                rot = 0.15
                lin = 0
                counter += 1
                print('Going on the wrong direction, turning around')
            else:
                rot = 0
                lin = 0
                counter = 0
                right_dir = True
                print('''I believe I'm on the right direction, resuming normal driving''')

        self.converter(lin, rot)

    def converter(self, lin, rot):
        left_motor = lin - rot
        right_motor = lin + rot
        self.driveMotors(left_motor, right_motor)

    def recorder(self, ground_sensor, ground_sensor_list, full_list):
        full_list.append(ground_sensor)
        if ground_sensor != -1:
            ground_sensor_list.append(ground_sensor)
        return ground_sensor_list, full_list

    def comparer(self, ground_sensor_list, full_list, default_list):
        curr_ground = ground_sensor_list[-1]
        prev_ground = ground_sensor_list[-2]
        prev_full = full_list[-2]
        default_ground_idx = int(default_list.index(curr_ground))

        if default_list[default_ground_idx - 1] == prev_ground:
            right_dir = True
        else:
            right_dir = False

        return right_dir




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
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
