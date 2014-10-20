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
def hello():
    print('hello')
    

class vector:
    x=0
    y=0
    z=0
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z

class iRobotTask1:
    __robot = 0
    __currentAngle = 0
    __currentPosition = 0
    __targetPosition = 0
    __rot = 0
    __state = 0 #
    
    __WHEEL_DISTANCE = 200.0 #mm
    def __init__(self, robot):
        self.__robot = robot
        self.__currentPosition =[0.0,0.0]
        self.__targetPosition = [0.0,0.0]
        self.__currentAngle = 0.0
        
    def update(self, dt, distance, angle):
        
        #calculate odometry = f(q, alpha)
        self.positionCalculate(distance, angle)
        
        dy = self.__targetPosition[1] - self.__currentPosition[1]
        dx = self.__targetPosition[0] - self.__currentPosition[0]
        print('HELLOjasouidhasuiodhasuidhausidhaiusdh'+str(degrees(atan2(dy, dx))))
        xi = atan2(dy, dx) - self.__currentAngle
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi
#         xi = 1
        norm = sqrt(dx**2 + dy**2)
#         norm = 100
        print('TASK1 UPDATE: '+str(dx)+' '+str(dy)+' '+str(degrees(xi))+' '+str(norm)+' '+str(degrees(self.__currentAngle)))
        
        if (norm > 50):
            
#             self.__robot.setWheelVel(300, -300)
            if (abs(xi) > radians(5)):
                # Rotation phase
                if (xi>pi):
                    xi-=pi
                elif(xi<-pi):
                    xi+=pi    
                vr = xi*self.__WHEEL_DISTANCE/(2*dt)
                vl = -vr
                self.__robot.setWheelVel(1*vr, 1*vl)
                print('CORRECTING angle')
            else:
                gama = 2*radians(angle)
                L = norm/(2*sin(radians(angle)))
                t1 = 1/500*(gama*L+gama*self.__WHEEL_DISTANCE/2)
                t2 = 1/500*(gama*L-gama*self.__WHEEL_DISTANCE/2)
                
                if (abs(t1)>abs(t2)):
                    t = t1
                else:
                    t = t2
                if (t < 0.2):
                    t = 0.2
                vl = (gama/t)*(L - self.__WHEEL_DISTANCE/2)
                vr = (gama/t)*(L + self.__WHEEL_DISTANCE/2)
                self.__robot.setWheelVel(vl,vr)
#                 # Translation phase
#                 pass
        else:
            self.__robot.setWheelVel(0, 0)
            # regulation ok
            pass
            
#         #if (rotation ok)?
#         if self.__state == 0:
#             alpha = atan2(self.__targetPosition[1], self.__targetPosition[0])
#             if (abs(alpha) > radians(5)):
#                 self.__state = 1
#             else:
#                 self.__state = 2
#         
#         elif self.__state == 1:
#             #Rotating phase
#             
#             self.positionCalculate()
#             
#             
#             
#         elif self.__state == 2:
#             #Travelling phase
#             pass
            

        
        
    def positionCalculate(self, distance, angle):
        # mm
        q = distance
        # rad 
        alpha = radians(angle)
        print('alpha: '+str(alpha))
        gama = 2*alpha
        
        # TODO: Calculate odometry by RADIUS !
#         if (gama):
#             L = q/gama
#             self.__currentPosition[0] -= L*sin(gama)
#             self.__currentPosition[1] += L-L*cos(gama)
#             self.__currentAngle += alpha
#             
#             angle = self.__currentAngle
#             if (angle<-pi):
#                 print('~~~~~~~~~~~~~~~~~~~~MINUS')
#                 self.__currentAngle += 2*pi
#             elif (angle>pi):
#                 print('~~~~~~~~~~~~~~~~~~~~PLUS')
#                 self.__currentAngle -= 2*pi
# #             self.__currentAngle %= 2*pi
#             
#         else:
        self.__currentPosition[0] += q*cos(self.__currentAngle)
        self.__currentPosition[1] += q*sin(self.__currentAngle)
        self.__currentAngle += alpha
        
        angle = self.__currentAngle
        if (angle<-pi):
            print('~~~~~~~~~~~~~~~~~~~~MINUS')
            self.__currentAngle += 2*pi
        elif (angle>pi):
            print('~~~~~~~~~~~~~~~~~~~~PLUS')
            self.__currentAngle -= 2*pi
    
        print('Current position: ', self.__currentPosition)
        
        
        
        
        
#         self.__currentAngle += dAngle
        
        # Calculate odometry
        
        # 
#         self.__currentPosition[0] += dDistance*cos(radians(self.__currentAngle))
#         self.__currentPosition[1] += dDistance*sin(radians(self.__currentAngle))
#         self.__currentAngle += dAngle
#         self.__currentAngle %= 360
#         print(str(self.__currentPosition[0]) +' '+str(self.__currentPosition[1]) +' '+str(self.__currentAngle) )

    
    def setTarget(self, targetX, targetY):
        self.__targetPosition = [targetX, targetY]


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
    def parseStream(self, data):
        i=1
        
#         for o in range(ord(data[0])):
#             print(ord(data[o]))
        while i < ord(data[0]):
            # PACKET ID
            pid = ord(data[i])
            
            # if you want to read more than one bytes, increment inside if
            i += 1 
            if pid == 7:
                self.__bumpRight = ord(data[i])&(0x01)
                self.__bumpLeft = (ord(data[i])&(0x02))>>1
                # print("Bump Left: " + str(self.__bumpLeft) + " Bump Right: " + str(self.__bumpRight()))
            elif pid == 8:
                __wall = ord(data[i])
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
                self.__IRByte = ord(data[i])
            elif pid == 18:
                pass
            elif pid == 19:
                if ord(data[i])&0x80:
                    self.__distance = -(65536 - (ord(data[i])*256 + ord(data[i+1])))
                else:
                    self.__distance = ord(data[i])*256 + ord(data[i+1])
#                 print("Distance: " + str(self.__distance))
#                 self.__distance = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 20:
#                 print(str(ord(data[i])) + str(ord(data[i+1])))
                if ord(data[i])&0x80:
                    self.__angle = -(65536 - (ord(data[i])*256 + ord(data[i+1])))
                else:
                    self.__angle = ord(data[i])*256 + ord(data[i+1])
#                 print("Angle: " + str(self.__angle))
#                 self.__angle = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 21:
                pass
            elif pid == 22:
                self.__voltage = ord(data[i])*256 + ord(data[i+1]) 
#                 print("Voltage: " + str(self.__voltage)) 
                i += 1
            elif pid == 23:
                if ord(data[i])&(0x80) != 0:    # negative current
                    self.__current = -1*((ord(data[i])&(0x7f))*256 + ord(data[i+1]))
                else:   # positive current
                    self.__current = ord(data[i])&(0x7f)*256 + ord(data[i+1])
#                 print("Current: " + str(self.__current)) 
                i += 1
            elif pid == 24:
                pass
            elif pid == 25:
                i += 1
            elif pid == 26:
                i += 1
            elif pid == 27:
#                 self.valueChanged.emit()
                
                self.__wallSignal = ord(data[i])<<8 + ord(data[i+1])
                # print("Wall signal: " + str(self.__wallSignal)) 
                i += 1
            elif pid == 28:
                i += 1
            elif pid == 29:
                i += 1
            elif pid == 30:
                i += 1
            elif pid == 31:
                i += 1
            elif pid == 32:
                pass
            elif pid == 33:
                i += 1
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
                i += 1
            elif pid == 40:
                i += 1
            elif pid == 41:
                i += 1
            elif pid == 42:
                i += 1
            else :
                print('ERROR  Wrong packet ID: ' + str(pid))
                return
                
            i += 1
     
class iRobot():
    __port = 0
    __packetLength = 0
#     __streamRunning = 0
    __packetLengthSemaphore = 0
    sensors = 0
    __currAngle = 0
    __currDistance = 0
    __task1 = 0
    __timerStream = 0
    __timerLastTime = 0
    __parent = 0
    def __init__(self, parent, path):
        self.sensors = iRobotSensors()
#       super(iRobot, self).__init__()
        self.__port = serial.Serial(path, baudrate=57600)
        self.__port.open()
        self.__packetLengthSemaphore = threading.Semaphore()
        self.__parent = parent
        

    def modeSafe(self):
#         self.__port.write("\x80\x83")
        self.__port.write(bytearray([0x80, 0x83]))
        
    def modeFull(self):
#         self.__port.write("\x80\x84")
        self.__port.write(bytearray([0x80, 0x84]))
        
    def goCircle(self):
        self.__port.write("\x89\x00\xff\x00\xff")
        
    def setWheelVel(self, left=0, right=0):
#         self.__port.write("\x91\x00\xff\x00\xff")
        left = int(left)
        right = int(right)

        leftMSB = (left&0xff00)>>8
        leftLSB = (left&0xff)
        
        rightMSB = (right&0xff00)>>8
        rightLSB = (right&0xff)
        
        self.__port.write(bytearray([0x91, leftMSB, leftLSB, rightMSB, rightLSB]))
    
    def stop(self):
        self.__port.write("\x91\x00\x00\x00\x00")
        self.__port.write("\x80")
 
    def streamStart(self):
        print('timing hello()')
#         self.__port.write(requestedPacketsString)
        self.__timerLastTime = time.time()
        threading.Timer(0.2, self.requestMoreData).start();
#         global TIMER
#         TIMER = threading.Timer(0.2, hello);
#         print('timing hello() END')
#         self.__port.write("\x94\x04\x07\x1B\x16\x17")
    
    
    def streamStop(self):
        pass
#         try:
#             self.__port.write("\x96\x00")
#         except:
#             pass
#         self.__streamRunning = False
#         global shuttingDown
#         shuttingDown = 1

#     def runTask(self, taskId):
#         self.__currentTask = taskId

    # Thread
    def streamRead(self):
#         self.__streamRunning = True
        global shuttingDown
        while not shuttingDown:
            try:
                print('READING OUT VALUES!')
#                 sync = ord(self.__port.read(1))
#                 if sync == 19:
                data = self.__port.read(4)
                print(str(ord(data[0])))
                print(str(ord(data[1])))
                print(str(ord(data[2])))
                print(str(ord(data[3])))


                # Do ti like angle!
                if ord(data[0])&0x80:
                    self.sensors.__distance = -(65536 - (ord(data[0])*256 + ord(data[1])))
                else:
                    self.sensors.__distance = ord(data[0])*256 + ord(data[1])
                print("Distance: " + str(self.sensors.__distance)) 

                angle = ord(data[2])*256 + ord(data[3])
                if angle>=32768:
                    self.sensors.__angle = angle - 65536
                else:
                    self.sensors.__angle = angle
                    
                print("Angle: " + str(self.sensors.__angle))
                
#                 self.sensors.parseStream(bytearray(0x06,0x13,data[0],data[1],0x14,data[2],data[3]))
                # Calculate dT
                curr_time = time.time();
                dt = curr_time - self.__timerLastTime;
                self.__timerLastTime = curr_time
                
                print('DAFUQMEGA')
                self.__parent.taskUpdate(dt, self.sensors.__distance, self.sensors.__angle)
                
            except:
                self.streamStop()
        print("iRobot Stream read thread Stopped")
    # Timer   
    def requestMoreData(self):
        print('request more data')
        self.__port.write(bytearray([0x95, 0x02, 0x13, 0x14]))
        
        global shuttingDown
        if not shuttingDown:
            threading.Timer(0.2, self.requestMoreData).start();
            
    def __del__(self):
        self.streamStop()
        self.stop()
        self.__port.close()
     
class mainWidget(QtGui.QWidget):
    __robot = 0
    __task1 = 0
    __currentTask = 0
    

    def __init__(self):
        super(mainWidget, self).__init__()
        
        uic.loadUi('iRobotGUI.ui',self)
        self.__robot = iRobot(self, "/dev/rfcomm0")

#         self.__robot.sensors.valueChanged.connect(self.slotSensorsChanged)
        self.__task1 = iRobotTask1(self.__robot)
        self.show()
        
    def slotStart(self):
        self.__robot.modeFull()
        global shuttingDown
        shuttingDown = 0
        print("iRobot Control Enabled")
    def slotStop(self):
        self.__robot.streamStop()
        self.__robot.stop()
        self.__currentTask = 0
        global shuttingDown
        shuttingDown = 1
        print("iRobot Control Disabled")
        
    def slotTask1Start(self):
        self.__robot.streamStart() # initialize timer for request more data

        self.__threadComm = threading.Thread(target=self.__robot.streamRead) # initialize reading stream
        self.__threadComm.start()
    def slotTask1Stop(self):
        self.__robot.streamStop()
    def slotTask1Go(self):
#         global timerStream
        self.__timerLastTime = time.time()
#         __timerStream = threading.Timer(0.5, self.__robot.requestMoreData);
#         __timerStream.start()
        targetX = int(self.task1LineEditTargetX.text())
        targetY = int(self.task1LineEditTargetY.text())
        self.__task1.setTarget(targetX, targetY)
        self.__currentTask = 1

        
    def slotSensorsChanged(self):
        print("iRobot Sensor data changed")
    

    def taskUpdate(self, dt, distance, angle):
        global shuttingDown
        # TODO:  Get angle and distance
        print('task update')
        if (self.__currentTask == 1):
            print('task update 1')
            self.__task1.update(dt, distance, angle)
            
            pass
#         print('ahoj')
#         curr_time = time.time();
#         dt = curr_time - self.__timerLastTime;
#         self.__timerLastTime = curr_time
#         if not shuttingDown:
#             self.__timerStream.interval = 0.2
#             self.__timerStream.run()
            


# timerStream = 0

app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

