#!/usr/bin/python
"""
Library for control iRobotCreate via bluetooth. 

Copyright (C) 2014  Amir Hammad, amir.hammad@hotmail.com
Copyright (C) 2014  Peter Belica, atmelatmel@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import serial
import threading
import sys
from math import *
from time import sleep  
import time
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import QObject, pyqtSignal

global shuttingDown
shuttingDown = 0

global TIMER
TIMER = 0

class iRobotTask1:
    __robot = 0
    __state = 0
    __MAXSTATES = 4
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0

    def reset(self):
        self.__state = 0
    def update(self):
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~update: '+str(self.__robot.sensors.angle()))
   
        p = 1000
        state = self.__state
        if (state == 0):
            self.__robot.positioning.setTarget(p, 0)
        elif (state == 1):
            self.__robot.positioning.setTarget(p, p)
        elif (state == 2):
            self.__robot.positioning.setTarget(0, p)
        elif (state == 3):
            self.__robot.positioning.setTarget(0, 0)
            
        if (self.__state < self.__MAXSTATES):
            if (self.__robot.positioning.targetOK()):
                self.__state += 1
                
class iRobotTask2:
    __robot = 0
    __state = 0
    __MAXSTATES = 5
    __spX = 0
    __spY = 0
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0

    def reset(self):
        self.__state = 0
        self.__spX = 0
        self.__spY = 0
    def update(self):
        WIDTH=300
        AREAWIDTH=1000
        AREAHEIGHT=1000

        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~update: '+str(self.__robot.sensors.angle()))
        if (self.__spY > AREAWIDTH):
            self.__robot.positioning.setTarget(0, 0)
            return
   
        if (self.__state < self.__MAXSTATES):
            if (self.__state != 0):
                if (self.__robot.positioning.targetOK()):
                    state = self.__state
                    if (state == 1):
                        self.__spX = AREAHEIGHT
                        self.__robot.positioning.setTarget(self.__spX, self.__spY)
                        self.__state += 1
                    elif (state == 2):
                        self.__spY += WIDTH
                        self.__robot.positioning.setTarget(self.__spX, self.__spY)
                        self.__state += 1
                    elif (state == 3):
                        self.__spX = 0 
                        self.__robot.positioning.setTarget(self.__spX, self.__spY)
                        self.__state += 1
                    elif (state == 4):
                        self.__spY += WIDTH
                        self.__robot.positioning.setTarget(self.__spX, self.__spY)
                        self.__state = 1
            else:
                self.__spX = AREAHEIGHT
                self.__robot.positioning.setTarget(self.__spX, self.__spY)
                self.__state = 2
            

class iRobotPositioning:
    __robot = 0
    __currentAngle = 0
    __currentPosition = 0
    __targetPosition = 0
    __rot = 0
    __state = 0 #
    __targetReached = 0
    __WHEEL_DISTANCE = 200.0 #mm
    def __init__(self, robot):
        self.__robot = robot
        self.__currentPosition =[0.0,0.0]
        self.__targetPosition = [0.0,0.0]
        self.__currentAngle = 0.0
        self.__targetReached = 0
        
    def regulate(self, dt):
        
#         #calculate odometry = f(q, alpha)
#         self.positionCalculate(distance, angle)
        distance = self.__robot.sensors.distance()
        angle = self.__robot.sensors.angle()
        dy = self.__targetPosition[1] - self.__currentPosition[1]
        dx = self.__targetPosition[0] - self.__currentPosition[0]
        print('HELLOjasouidhasuiodhasuidhausidhaiusdh'+str(degrees(atan2(dy, dx))))
        
        # Error between desired angle and current angle
        xi = atan2(dy, dx) - self.__currentAngle
        
        # Saturate error angle
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi

#         print('TASK1 UPDATE: '+str(dx)+' '+str(dy)+' '+str(degrees(xi))+' '+str(degrees(self.__currentAngle)))
        print('DT: '+str(dt))
        if (not self.targetOK()):
            if (abs(xi) > radians(5)):
                # Rotation phase
                
                vr = 100*xi+50*copysign(1,xi) #*self.__WHEEL_DISTANCE/(2*dt)
                vl = -vr
                self.__robot.setWheelVel(vr, vl)
                print('CORRECTING angle')
            else:               
                self.__robot.setWheelVel(300,300)
        else:
            self.__robot.setWheelVel(0, 0)
            # regulation ok
            pass


        
        
    def positionCalculate(self):
        distance = self.__robot.sensors.distance()
        angle = self.__robot.sensors.angle()
        # mm
        q = distance
        # rad 
        alpha = radians(angle)
        
        self.__currentPosition[0] += q*cos(self.__currentAngle)
        self.__currentPosition[1] += q*sin(self.__currentAngle)
        self.__currentAngle += alpha
        
        angle = self.__currentAngle
        if (angle<-pi):
            self.__currentAngle += 2*pi
        elif (angle>pi):
            self.__currentAngle -= 2*pi
    
        print('Current position: ', self.__currentPosition)
    
    def setTarget(self, targetX, targetY):
        self.__targetPosition = [targetX, targetY]
    
    def targetOK(self):
        dy = self.__targetPosition[1] - self.__currentPosition[1]
        dx = self.__targetPosition[0] - self.__currentPosition[0]
        norm = sqrt(dx**2 + dy**2)
        
        if (norm<50):
            return 1
        else:
            return 0


class iRobotSensors:
    __bumpLeft = 0
    __bumpRight = 0
    __wall = 0
    __IRByte = 0
    __distance = 0
    __angle = 0
    __voltage = 0
    __current = 0
    __wallSignal = 0
    valueChanged = pyqtSignal()
    
    def bumpLeft(self):
        return self.__bumpLeft
    def bumpRight(self):
        return self.__bumpRight
    def wall(self):
        return self.__wall
    def IRByte(self):
        return self.__IRByte
    def distance(self):
        return self.__distance
    def angle(self):
        return self.__angle
    def voltage(self):
        return self.__voltage()
    def current(self):
        return self.__current()
    def wallSignal(self):
        return self.__wallSignal()
    
    # byte 0: LENGTH
    def parse(self, id, data):
        # PACKET ID
        pid = id
         
        if pid == 7:
#             self.__bumpRight = ord(data[i])&(0x01)
#             self.__bumpLeft = (ord(data[i])&(0x02))>>1
            # print("Bump Left: " + str(self.__bumpLeft) + " Bump Right: " + str(self.__bumpRight()))
            pass
        elif pid == 8:
#             __wall = ord(data[i])
            pass
        elif pid == 9:
            pass
        elif pid == 10:
            pass
        elif pid == 11:
            pass
        elif pid == 12:
            pass
        elif pid == 13:
            pass
        elif pid == 14:
            pass
        elif pid == 15:
            pass
        elif pid == 16:
            pass
        elif pid == 17:
#             self.__IRByte = ord(data[i])
            pass
        elif pid == 18:
            pass
        elif pid == 19:
            distance = data[0]*256 + data[1]
            if distance >= 32768:
                self.__distance = distance - 65536
            else:
                self.__distance = distance
            
        elif pid == 20:
            angle = data[0]*256 + data[1]
            if angle >= 32768:
                self.__angle = angle - 65536
            else:
                self.__angle = angle
                 
        elif pid == 21:
            pass
        elif pid == 22:
#             self.__voltage = ord(data[i])*256 + ord(data[i+1]) 
#                 print("Voltage: " + str(self.__voltage)) 
            pass
        elif pid == 23:
#             if ord(data[i])&(0x80) != 0:    # negative current
#                 self.__current = -1*((ord(data[i])&(0x7f))*256 + ord(data[i+1]))
#             else:   # positive current
#                 self.__current = ord(data[i])&(0x7f)*256 + ord(data[i+1])
#                 print("Current: " + str(self.__current)) 
            pass
        elif pid == 24:
            pass
        elif pid == 25:
            pass
        elif pid == 26:
            pass
        elif pid == 27:
#                 self.valueChanged.emit()
            
#             self.__wallSignal = ord(data[i])<<8 + ord(data[i+1])
            # print("Wall signal: " + str(self.__wallSignal)) 
            pass
        elif pid == 28:
            pass
        elif pid == 29:
            pass
        elif pid == 30:
            pass
        elif pid == 31:
            pass
        elif pid == 32:
            pass
        elif pid == 33:
            pass
        elif pid == 34:
            pass
        elif pid == 35:
            pass
        elif pid == 36:
            pass
        elif pid == 37:
            pass
        elif pid == 38:
            pass
        elif pid == 39:
            pass
        elif pid == 40:
            pass
        elif pid == 41:
            pass
        elif pid == 42:
            pass
        else :
            print('ERROR  Wrong packet ID: ' + str(pid))
            return
                
class lowPass():
    __data = 0
    def __init__(self, data = 0):
        self.__data = data
        
    def data(self, data, dt, timeConstant = 0):
        if ((dt <= 0) or (timeConstant<=0)): 
            return data
        C = dt/(timeConstant+dt)
        self.__data = (1-C)*self.__data + C*data
        return self.__data
        
class iRobot():
    __port = 0
    __packetLength = 0
    __packetLengthSemaphore = 0
    sensors = 0
    positioning = 0
    __currAngle = 0
    __currDistance = 0
    __task1 = 0
    __timerStream = 0
    __timerLastTime = 0
    __timerLastTimeLowPass = 0
    __parent = 0
    __leftWheelVelTarget = 0
    __rightWheelVelTarget = 0
    
    __prescaler = 0
    def __init__(self, parent, path):
        self.sensors = iRobotSensors()

        self.__port = serial.Serial(path, baudrate=57600)
        self.__port.open()
        self.__packetLengthSemaphore = threading.Semaphore()
        self.__parent = parent
        self.__leftWheelVelLP = lowPass(0)
        self.__rightWheelVelLP = lowPass(0)
        self.positioning = iRobotPositioning(self) 

    def modeSafe(self):
#         self.__port.write("\x80\x83")
        self.__port.write(bytearray([0x80, 0x83]))
        
    def modeFull(self):
#         self.__port.write("\x80\x84")
        self.__port.write(bytearray([0x80, 0x84]))
        
    def goCircle(self):
        self.__port.write("\x89\x00\xff\x00\xff")
        
    def setWheelVel(self, right=0, left=0):
        self.__leftWheelVelTarget = int(left)
        self.__rightWheelVelTarget = int(right)
        self.sendWheelVel()
    def sendWheelVel(self):
        
        left = self.__leftWheelVelTarget
        right = self.__rightWheelVelTarget
        
        leftMSB = (left&0xff00)>>8
        leftLSB = (left&0xff)
        
        rightMSB = (right&0xff00)>>8
        rightLSB = (right&0xff)
        
        self.__port.write(bytearray([0x91,  rightMSB, rightLSB, leftMSB, leftLSB]))
    
    def stop(self):
        self.__port.write("\x91\x00\x00\x00\x00")
        self.__port.write("\x80")
 
    def streamStart(self):

        now = time.time()
        self.__timerLastTime = now
        self.__timerLastTimeLowPass = now
        self.__prescaler = 10
        self.positioning.setTarget(0, 0)
        threading.Timer(0.2, self.requestMoreData).start(); 
    
    def streamStop(self):
        pass

    # Thread
    def streamRead(self):
        global shuttingDown
        while not shuttingDown:
            try:
                print('READING OUT VALUES!')

                data = self.__port.read(4)
                
                self.sensors.parse(0x13, bytearray([ord(data[0]), ord(data[1])]))
                self.sensors.parse(0x14, bytearray([ord(data[2]), ord(data[3])]))
#                 self.sensors.parseStream(bytearray(0x06,0x13,data[0],data[1],0x14,data[2],data[3]))
                # Calculate dT
                curr_time = time.time();
                dt = curr_time - self.__timerLastTime;
                self.__timerLastTime = curr_time
                
                self.positioning.positionCalculate()
                self.positioning.regulate(dt)
                
                self.__parent.taskUpdate(dt)
                
            except:
                self.streamStop()
        print("iRobot Stream read thread Stopped")
    # Timer   
    def requestMoreData(self):
        # Send WheelVelocity
        
#         self.sendWheelVel()
#         if (self.__prescaler <= 0):
#             self.__prescaler = 10
            # Prescaler for 200ms
        print('request more data')
        self.__port.write(bytearray([0x95, 0x02, 0x13, 0x14]))
        global shuttingDown
        if not shuttingDown:
            threading.Timer(0.2, self.requestMoreData).start();
#         else:
#             self.__prescaler -= 1
            
          
            
    def __del__(self):
        self.streamStop()
        self.stop()
        self.__port.close()
     
class mainWidget(QtGui.QWidget):
    robot = 0
    __task1 = 0
    __task2 = 0
    __currentTask = 0
    

    def __init__(self):
        super(mainWidget, self).__init__()
        
        uic.loadUi('iRobotGUI.ui',self)
        self.robot = iRobot(self, "/dev/rfcomm3")

#         self.__robot.sensors.valueChanged.connect(self.slotSensorsChanged)
        self.__task1 = iRobotTask1(self.robot)
        self.__task2 = iRobotTask2(self.robot)
        self.show()
        
    def slotStart(self):
        self.robot.modeFull()
        global shuttingDown
        shuttingDown = 0
        print("iRobot Control Enabled")
    def slotStop(self):
        self.robot.streamStop()
        self.robot.stop()
        self.__currentTask = 0
        global shuttingDown
        shuttingDown = 1
        print("iRobot Control Disabled")
        
    def slotTask1Start(self):
        self.robot.streamStart() # initialize timer for request more data

        self.__threadComm = threading.Thread(target=self.robot.streamRead) # initialize reading stream
        self.__threadComm.start()
        
    def slotTask1Stop(self):
        self.robot.streamStop()
        
    def slotTask1Go(self):
        self.__timerLastTime = time.time()
        self.__task1.reset()
        self.__currentTask = 1
        
    def slotTask2Go(self):
        self.__timerLastTime = time.time()
        self.__task2.reset()
        self.__currentTask = 2
        
    def slotSensorsChanged(self):
        print("iRobot Sensor data changed")
    

    def taskUpdate(self, dt):
        global shuttingDown
        # TODO:  Get angle and distance
        print('task regulate')
        if (self.__currentTask == 1):
            print('task regulate 1')
            self.__task1.update()
        elif (self.__currentTask == 2):
            print('task regulate 2')
            self.__task2.update()        

app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

