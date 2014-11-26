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
from PyQt4 import QtCore, QtGui, uic, Qt
from PyQt4.QtCore import QObject, pyqtSignal
from PyQt4.QtGui import *
from PyQt4.Qt import QMutex, QEvent

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
                
        self.__robot.positioning.regulate()
                
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
            self.__robot.positioning.regulate()
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
                
        self.__robot.positioning.regulate()
            
class iRobotTask3:
    __robot = 0
    __state = 0
    __lastCalculateTime = 0
    __STEP = 150
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0
        self.__target = [0,0]

    def reset(self):
        self.__state = 0
        
    def setTarget(self, x, y):
        self.__target = [x, y]
        
    def update(self):
        bl = self.__robot.sensors.bumpLeft()
        br = self.__robot.sensors.bumpRight()
        state = self.__state
        [cx, cy] = self.__robot.positioning.getPosition()
#         cx = 0
#         cy = 0
        ca = self.__robot.positioning.getAngle()
        wall = self.__robot.sensors.wallSignal()
#         wall=0
        print ('state!!!!' + str(state))
        if (state == 0):
            if (bl and br):
                self.__state = 11           
                self.__robot.setWheelVel(-50, -50)
#                 self.__robot.positioning.setTarget(cx, cy)
                print ('nulujem rychlost')
                self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            elif (bl):
                self.__state = 21
                self.__robot.setWheelVel(-50, -50)
#                 self.__robot.positioning.setTarget(cx, cy)
                self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            elif (br):
                self.__state = 31
                self.__robot.setWheelVel(-50, -50)
#                 self.__robot.positioning.setTarget(cx, cy)
                self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            else:
                self.__robot.positioning.setTarget(self.__target[0], self.__target[1])
                self.__robot.positioning.regulate()
        elif (state == 11):
            # turn left + 100mm
            if (self.__lastCalculateTime +0.5 < self.__robot.positioning.getLastCalculateTime()):
                self.__state = 12

                uhol = radians(90)
                tx = cx + self.__STEP*cos(ca+uhol)
                ty = cy + self.__STEP*sin(ca+uhol)
                
                self.__robot.positioning.setTarget(tx,ty)
                
        elif (state == 12):
            if (self.__robot.positioning.targetOK()):
                print ('target ok')
                if (wall > 10):
                    tx = cx + self.__STEP * cos(ca)
                    ty = cy + self.__STEP * sin(ca)
                    self.__robot.positioning.setTarget(tx,ty)
                else:
                    # CHOD NA CIEL
                    self.__state = 0
                    self.__robot.positioning.setTarget(self.__target[0], self.__target[1])
            else:
                self.__robot.positioning.regulate()
        elif (state == 21):
            if (self.__lastCalculateTime +0.5 < self.__robot.positioning.getLastCalculateTime()):
                self.__state = 32
#                 tx = cx - 100*sin(ca)
#                 ty = cy + 100*cos(ca)
                uhol = radians(135)
                tx = cx + self.__STEP*cos(ca+uhol)
                ty = cy + self.__STEP*sin(ca+uhol)
                self.__robot.positioning.setTarget(tx,ty)
            
        elif (state == 31):
            if (self.__lastCalculateTime +0.5 < self.__robot.positioning.getLastCalculateTime()):
                self.__state = 32
#                 tx = cx - 100*sin(ca)
#                 ty = cy + 100*cos(ca)
                uhol = radians(45)
                tx = cx + self.__STEP*cos(ca+uhol)
                ty = cy + self.__STEP*sin(ca+uhol)
                self.__robot.positioning.setTarget(tx,ty)
                
        elif (state == 32):
            if (self.__robot.positioning.targetOK()):
                
                if (wall > 10):
                    tx = cx + self.__STEP * cos(ca)
                    ty = cy + self.__STEP * sin(ca)
                    self.__robot.positioning.setTarget(tx,ty)
                else:
                    # CHOD NA CIEL
                    self.__state = 0
                    self.__robot.positioning.setTarget(self.__target[0], self.__target[1])
            else:
                self.__robot.positioning.regulate()
   
                        
class iRobotTask4:
    __robot = 0
    __state = 0
    __desiredAngle = 0
    __backwardDistance = 0
    __forwardDistance = 0
    __maxWallSignal = 0
    __MAXSTATES = 4
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0

    def reset(self):
        self.__state = 0
        
    def cureAngleDisease(self, xi):
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi
        return xi
    def update(self):
        bl = self.__robot.sensors.bumpLeft()
        br = self.__robot.sensors.bumpRight()
        wall = self.__robot.sensors.wallSignal()
        distance = self.__robot.sensors.distance()
        
        if (bl and br):
            xi = self.cureAngleDisease(radians(90) + self.__robot.positioning.getCurrentAngle())
            self.__desiredAngle = xi
            self.__backwardDistance = 0
            self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            self.__state = 1           
        elif (bl):
            xi = self.cureAngleDisease(radians(135) + self.__robot.positioning.getCurrentAngle())
            self.__desiredAngle = xi
            self.__backwardDistance = 0
            self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            self.__state = 1
        elif (br):
            xi = self.cureAngleDisease(radians(45) + self.__robot.positioning.getCurrentAngle())
            self.__desiredAngle = xi
            self.__backwardDistance = 0
            self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            self.__state = 1
            
        if (self.__state == 1):   #Go back
            if (distance<0):
                if (self.__lastCalculateTime < self.__robot.positioning.getLastCalculateTime()):
                    self.__backwardDistance += distance 
                    self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            if (self.__backwardDistance < -10):
                self.__robot.setWheelVel(0,0)
                if (self.__maxWallSignal > 40):
                    self.__maxWallSignal = 0
                else:
                    self.__maxWallSignal = 0
                self.__state = 2
            else:
                print("BACKWARD DISTANCE" + str(self.__backwardDistance))
                self.__robot.setWheelVel(-30, -30)
                
                
        elif (self.__state == 2):   #Rotate
            currentAngle = self.__robot.positioning.getCurrentAngle()
            if (abs(self.cureAngleDisease(self.__desiredAngle - currentAngle)) < radians(5)):
                self.__robot.setWheelVel(0,0)
                self.__state = 3
            else:
                if (wall < self.__maxWallSignal-20):
                    self.__robot.setWheelVel(0,0)
                    self.__state = 3
                else:
                    if (wall > self.__maxWallSignal):
                        self.__maxWallSignal = wall
                    print ("\t\t\t MAX WALL SIGNAL" + str(self.__maxWallSignal))
                    self.__robot.positioning.regulateAngle(self.__desiredAngle)
                
                
        elif (self.__state == 3):   #Go along wall
            wall = self.__robot.sensors.wallSignal()
            vbase = 100
            regP = 1
            desiredWall = 40
            error = desiredWall - wall
            rot = regP * error
            vl = vbase + rot
            vr = vbase - rot
            self.__robot.setWheelVel(vr, vl)
            if (wall < 1):
                self.__forwardDistance = 0
                self.__state = 4

                    
                
                
        elif (self.__state == 4):   # Go straight N milimeters
            if (self.__lastCalculateTime < self.__robot.positioning.getLastCalculateTime()):
                self.__forwardDistance += distance 
                self.__lastCalculateTime = self.__robot.positioning.getLastCalculateTime()
            
            if (self.__forwardDistance > 350):
                self.__robot.setWheelVel(0, 0)
                xi = self.cureAngleDisease(radians(-107.5) + self.__robot.positioning.getCurrentAngle())
                self.__desiredAngle = xi
                self.__state = 5
            else:
                self.__robot.setWheelVel(100, 100)
                
                
        elif (self.__state == 5):
            currentAngle = self.__robot.positioning.getCurrentAngle()
            if (abs(self.cureAngleDisease(self.__desiredAngle - currentAngle)) < radians(5)):
                self.__robot.setWheelVel(0,0)
                self.__state = 0
            else:
                self.__robot.positioning.regulateAngle(self.__desiredAngle)
        else:
            wall = self.__robot.sensors.wallSignal()
            if (wall > 5):
                self.__state = 3
            self.__robot.positioning.goStraight()


class iRobotTask5:
    __robot = 0
    __state = 0
    __R_last = 0
    __G_last = 0
    __F_last = 0
    __posIncoming = 0
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0
        self.__posIncoming = [0,0]

    def reset(self):
        self.__state = 0
        self.__navState = 0
        self.__regulate = 0
        self.__R_last = 0
        self.__G_last = 0
        self.__F_last = 0
        self.__countR = 0
        self.__countG = 0
        self.__countF = 0
        self.__posIncoming = [0,0]
        
    def __parseIrRed(self, irbyte):
        if (irbyte == 248 or irbyte == 252 or irbyte == 250 or irbyte == 254):
            return 1
        else:
            return 0
    def __parseIrGreen(self, irbyte):
        if (irbyte == 244 or irbyte == 252 or irbyte == 246 or irbyte == 254):
            return 1
        else:
            return 0
    def __parseIrForce(self, irbyte):
        if (irbyte == 242 or irbyte == 250 or irbyte == 246 or irbyte == 254):
            return 1
        else:
            return 0
        
    # stavy: 
    def __navigationControl(self, R, G, F, R_last, G_last, F_last, currPos):
        print ('nehehe X1')
        if (self.__regulate == 0):
            if (R and G):
                return 1
            
            if ((not R and R_last) or (not G and G_last) or (not(G or R))):
                if (self.__navState == 0):
                    tmpWP = [(currPos[0] + self.__posIncoming[0])/2,
                                       (currPos[1] + self.__posIncoming[1])/2]
                    diffX = currPos[0] - self.__posIncoming[0]
                    diffY = currPos[1] - self.__posIncoming[1]
                    self.__vectorToOutcoming = atan2(diffY, diffX)
                    self.__waypoint = [cos(self.__vectorToOutcoming)*12 + tmpWP[0], sin(self.__vectorToOutcoming)*12+tmpWP[1]]
                self.__robot.positioning.setTarget(self.__waypoint[0], self.__waypoint[1])
                self.__navState = self.__navState + 1
                self.__regulate = 1
        
        
        
        if (self.__regulate == 1):#move
            if (self.__robot.positioning.targetOK()):
                self.__regulate = 2
            else:
                self.__robot.positioning.regulate()
        elif (self.__regulate == 2):#rot
            
            if (self.__navState == 1):
                des = self.__vectorToOutcoming + radians(-90)
            elif (self.__navState == 2):
                des = self.__vectorToOutcoming + radians(90)
            elif (self.__navState == 3):
                des = self.__vectorToOutcoming + radians(135)
            elif (self.__navState == 4):
                des = self.__vectorToOutcoming + radians(-45)
            elif (self.__navState == 5):
                des = self.__vectorToOutcoming + radians(-135)
            elif (self.__navState == 6):
                des = self.__vectorToOutcoming + radians(45)
                
            if (self.__robot.positioning.getAngleError(des) > radians(5)):
                self.__robot.positioning.regulateAngle(des)
            else:
                # skoc
                self.__regulate = 0
#                 if (R or G):
                self.__robot.setWheelVel(60, 60)
#                 else:
#                     self.__robot.setWheelVel(0, 0)
            
            
        
    def update(self):
        irbyte = self.__robot.sensors.IRByte()
        R_last = self.__R_last
        G_last = self.__G_last
        F_last = self.__F_last
        R = self.__parseIrRed(irbyte)
        G = self.__parseIrGreen(irbyte)
        F = self.__parseIrForce(irbyte)
        
        
        if (R != R_last):
            if (self.__countR < 10):
                R = R_last
                self.__countR = self.__countR + 1
        else:
            self.__countR = 0
            
        if (G != G_last):
            if (self.__countG < 10):
                G = G_last
                self.__countG = self.__countG + 1
        else:
            self.__countG = 0
            
        if (F != F_last):
            if (self.__countF < 10):
                F = F_last
                self.__countF = self.__countF + 1
        else:
            self.__countF = 0
            
        
            
        currPos = self.__robot.positioning.getPosition()
        currAngle = self.__robot.positioning.getAngle()
        
        if (self.__state == 0):
            if ((R and not R_last) or (G and not G_last)):
                print ('\t\t\t\t STAF 0')
                self.__posIncoming = [currPos[0],currPos[1]]
                print (self.__posIncoming)
                self.__state = 1
                
            self.__robot.setWheelVel(100, 100)
            
        elif (self.__state == 1):         
            if (self.__navigationControl(R, G, F, R_last, G_last, F_last, currPos)):
                # mame R and G
                self.__robot.setWheelVel(0,0)
                self.__state = 2
            
                print ('\t\t\t\t JUPPI')
                
#             self.__robot.positioning.goStraight()
        self.__R_last = R
        self.__G_last = G
        self.__F_last = F
        
class iRobotTask6:
    __robot = 0
    __keyState = 0
    def __init__(self, robot):
        self.__robot = robot
        self.__state = 0

    def reset(self):
        self.__state = 0
        
    def setKeyState(self, keyState):
        self.__keyState = keyState
        
    def update(self):
        baseSpeed = 200
        baseRotation = 50
        keyState = self.__keyState
        # 1,  2     4     8
        # Up, Down, Left, Right
        outL = 0
        outR = 0
        if (keyState == 1):
            outL = baseSpeed
            outR = baseSpeed
        elif (keyState == 2):
            outL = -baseSpeed
            outR = -baseSpeed
        elif (keyState == 5):
            outL = baseSpeed - baseRotation
            outR = baseSpeed + baseRotation
        elif (keyState == 6):
            outL = -baseSpeed + baseRotation
            outR = -baseSpeed - baseRotation
        elif (keyState == 9):
            outL = baseSpeed + baseRotation
            outR = baseSpeed - baseRotation
        elif (keyState == 10):
            outL = -baseSpeed - baseRotation
            outR = -baseSpeed + baseRotation
            
        self.__robot.setWheelVel(outR,outL)
class iRobotPositioning:
    __robot = 0
    __currentAngle = 0
    __currentPosition = 0
    __targetPosition = 0
    __rot = 0
    __state = 0 #
    __WHEEL_DISTANCE = 200.0 #mm
    __lastCalculateTime = 0
    def __init__(self, robot):
        self.__robot = robot
        self.__currentPosition =[0.0,0.0]
        self.__targetPosition = [0.0,0.0]
        self.__currentAngle = 0.0
       
    def reinit(self):
        self.__currentPosition =[0.0,0.0]
        self.__currentAngle = 0.0 
    def regulate(self):
        
#         #calculate odometry = f(q, alpha)
#         self.positionCalculate(distance, angle)
#         distance = self.__robot.sensors.distance()
#         angle = self.__robot.sensors.angle()
        dy = self.__targetPosition[1] - self.__currentPosition[1]
        dx = self.__targetPosition[0] - self.__currentPosition[0]
        
        # Error between desired angle and current angle
        xi = atan2(dy, dx) - self.__currentAngle
        
        # Saturate error angle
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi

#         print('TASK1 UPDATE: '+str(dx)+' '+str(dy)+' '+str(degrees(xi))+' '+str(degrees(self.__currentAngle)))
        if (not self.targetOK()):
            if (abs(xi) > radians(5)):
                # Rotation phase
                
                vr = 100*xi+50*copysign(1,xi)
                vl = -vr
                self.__robot.setWheelVel(vr, vl)
                print('CORRECTING angle')
            else:               
                self.__robot.setWheelVel(300,300)
        else:
            self.__robot.setWheelVel(0, 0)
            # regulation ok
            pass


    def regulateAngle(self, desiredAngle):
        # Error between desired angle and current angle
        xi = desiredAngle - self.__currentAngle
        
        # Saturate error angle
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi
        
        vr = 100*xi+50*copysign(1,xi)
        vl = -vr
        self.__robot.setWheelVel(vr, vl)

    
    def cureAngleDisease(self, xi):
        if (xi<-pi):
            xi += 2*pi
        elif (xi>pi):
            xi -= 2*pi
        return xi
    
    def getAngleError(self, desiredAngle):
        xi = desiredAngle - self.__currentAngle
        xi = self.cureAngleDisease(xi)          
        return abs(xi)
    def getCurrentAngle(self):
        return self.__currentAngle
        
    def goStraight(self):
        self.__robot.setWheelVel(200, 200)
        
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
        
        self.__lastCalculateTime = time.time()
        
        print('Current position: ', self.__currentPosition)

    def getLastCalculateTime(self):
        return self.__lastCalculateTime
    
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
        
    def getPosition(self):
        return self.__currentPosition
    def getAngle(self):
        return self.__currentAngle


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
    __rightWheelOC = 0
    __leftWheelOC = 0
    __temperature = 0
    __charge = 0
    __capacity = 0
    
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
        return self.__voltage
    def current(self):
        return self.__current
    def wallSignal(self):
        return int(self.__wallSignal)
    def leftWheelOC(self):
        return self.__leftWheelOC
    def rightWheelOC(self):
        return self.__rightWheelOC
    def temperature(self):
        return self.__temperature
    def charge(self):
        return self.__charge
    def capacity(self):
        return self.__capacity

    # byte 0: LENGTH
    def parse(self, pid, data):
        # PACKET ID
#         print("DATA: "+str(data[0]))
        if pid == 7:
#             print(str(data[0]))
            self.__bumpRight = int(data[0]&(0x01))
            self.__bumpLeft = int(data[0]&(0x02))>>1
#             print("Bump Left: " + str(self.bumpLeft()) + " Bump Right: " + str(self.bumpRight()))
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
#             print("Virtually virtual wall")
            print("Virtual wall: "+str(data[0]))
#             print("Virtually virtual wallX")
            pass
        elif pid == 14:
            self.__rightWheelOC = int(data[0]&(8))
            self.__leftWheelOC = int(data[0]&(16))
        elif pid == 15:
            pass
        elif pid == 16:
            pass
        elif pid == 17:
            self.__IRByte = data[0]
            print("\t\t\t\t\t\t\t#### IRBYTE"+str(self.__IRByte))
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
            self.__voltage = data[0]*256 + data[1] 
#                 print("Voltage: " + str(self.__voltage)) 
            pass
        elif pid == 23:
            current = data[0]*256 + data[1]
            if current >= 32768:
                self.__current = current - 65536
            else:
                self.__current = current

        elif pid == 24:
            temp = data[0]
            if temp >= 128:
                self.__temperature = temp - 256
            else:
                self.__temperature = temp
        elif pid == 25:
            self.__charge = data[0]*256 + data[1] 
        elif pid == 26:
            self.__capacity = data[0]*256 + data[1]
        elif pid == 27:
#                 self.valueChanged.emit()
            
            self.__wallSignal = data[0]*256 + data[1]
#             print("\t\t\t\tWall signal: " + str(self.__wallSignal)) 
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
    
    __requestDataSelection = 0
    __lastOdometryTime = 0
    __prescaler = 0
    
    def __init__(self, parent, path):
        self.sensors = iRobotSensors()

        self.__port = serial.Serial(path, baudrate=57600)
        self.__port.open()
        self.__port.flushInput()
        self.__packetLengthSemaphore = threading.Semaphore()
        self.__parent = parent
        self.__leftWheelVelLP = lowPass(0)
        self.__rightWheelVelLP = lowPass(0)
        self.positioning = iRobotPositioning(self) 

    def modeSafe(self):
#         self.__port.write("\x80\x83")
        self.__port.write(bytearray([0x80, 0x83]))
        
    def actualLeftWheelVel(self):
        return self.__leftWheelVelTarget
    
    def actualRightWheelVel(self):
        return self.__rightWheelVelTarget
        
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
        self.__prescaler = 0
        self.positioning.setTarget(0, 0)
        threading.Timer(0.04, self.requestMoreData).start(); 
    
    def streamStop(self):
        pass

    # Thread
    def streamRead(self):
        global shuttingDown
        while not shuttingDown:
            try:

                
                if (self.__requestDataSelection == 1):
#                     print(':-o 1')
                    data = self.__port.read(19)
#                     print('READING OUT VALUES! 1')
                    self.sensors.parse(0x13, bytearray([ord(data[0]), ord(data[1])]))
                    self.sensors.parse(0x14, bytearray([ord(data[2]), ord(data[3])]))
                    self.sensors.parse(0x07, bytearray([ord(data[4])]))
                    self.sensors.parse(0x1B, bytearray([ord(data[5]), ord(data[6])]))
                    self.sensors.parse(0x0D, bytearray([ord(data[7])]))
                    self.sensors.parse(0x11, bytearray([ord(data[8])]))
                    self.sensors.parse(0x0e, bytearray([ord(data[9])]))
                    self.sensors.parse(0x16, bytearray([ord(data[10]), ord(data[11])]))
                    self.sensors.parse(0x17, bytearray([ord(data[12]), ord(data[13])]))
                    self.sensors.parse(0x18, bytearray([ord(data[14])]))
                    self.sensors.parse(0x19, bytearray([ord(data[15]), ord(data[16])]))
                    self.sensors.parse(0x1a, bytearray([ord(data[17]), ord(data[18])]))
                    self.positioning.positionCalculate()
                    self.__parent.taskUpdate()
                    self.__requestDataSelection = 0
                elif (self.__requestDataSelection == 2):
#                     print(':-o 2')
                    data = self.__port.read(5)
#                     print('READING OUT VALUES! 2' + str(ord(data[0])))
                    self.sensors.parse(0x07, bytearray([ord(data[0])]))
                    self.sensors.parse(0x1B, bytearray([ord(data[1]), ord(data[2])]))
                    self.sensors.parse(0x0D, bytearray([ord(data[3])]))
                    self.sensors.parse(0x11, bytearray([ord(data[4])]))
                    self.__parent.taskUpdate()
                    self.__requestDataSelection = 0
                else: 
                    continue
#                 self.sensors.parseStream(bytearray(0x06,0x13,data[0],data[1],0x14,data[2],data[3]))

                
                
                
                
                
            except:
                self.streamStop()
        print("iRobot Stream read thread Stopped")
    # Timer   
    def requestMoreData(self):
        # Send WheelVelocity
        
#         self.sendWheelVel()
        if (self.__lastOdometryTime >= 0.2):
            self.__lastOdometryTime -= 0.2
            # Prescaler for 200ms
            print('request more data')
            if (self.__requestDataSelection == 0):
                self.__requestDataSelection = 1
                self.__port.write(bytearray([0x95, 12, 0x13, 0x14, 0x07, 0x1B, 0x0D, 0x11, 0x0e, 0x16, 0x17, 0x18, 0x19, 0x1A]))
        else:
            # Prescaler for 40ms
            print('HUMP & BUMP')
            if (self.__requestDataSelection == 0):
                self.__requestDataSelection = 2
                self.__port.write(bytearray([0x95, 0x04, 0x07, 0x1B, 0x0D, 0x11]))
            self.__prescaler -= 1

        
        global shuttingDown
        if not shuttingDown:
                # Calculate dT
            curr_time = time.time();
            dt = curr_time - self.__timerLastTime;
            self.__timerLastTime = curr_time
            self.__lastOdometryTime += dt
            threading.Timer(0.04, self.requestMoreData).start();
            
          
            
    def __del__(self):
        self.streamStop()
        self.stop()
        self.__port.close()
class CMButton(QtGui.QPushButton):
    __keyState = 0
    def __init__(self, arg):
        super(CMButton, self).__init__(arg)
        
    def keyState(self):
        return self.__keyState
    def mPressed(self, key, state):
         
        if (not key.isAutoRepeat()):
            k = key.key()
            i = -1
            if (k==QtCore.Qt.Key_Up):
                i = 0
            elif (k == QtCore.Qt.Key_Down):
                i = 1
            elif (k == QtCore.Qt.Key_Left):
                i = 2
            elif (k == QtCore.Qt.Key_Right):
                i = 3
            if (i >= 0):
                if (state):
                    self.__keyState |= (1<<i)
                else:
                    self.__keyState &= ~(1<<i)
                print(self.keyState())
                
    def keyPressEvent(self, key):
        self.mPressed(key, True);
        key.accept()
    
    def keyReleaseEvent(self, key):
        self.mPressed(key, False);
        key.accept()

class mainWidget(QtGui.QWidget):
    robot = 0
    __currentTask = 0
    trigger = pyqtSignal()
    def rightWheelOvercurrent(self, on):
        paleta = self.label_7.palette()
        
        if (on):            
            paleta.setColor(self.label_7.backgroundRole(), QColor.fromRgbF(1, 0, 0, alpha=1))
        else:
            paleta.setColor(self.label_7.backgroundRole(), QColor.fromRgbF(0, 1, 0, alpha=1))
        self.label_7.setPalette(paleta)
        
    def leftWheelOvercurrent(self, on):
        paleta = self.label_9.palette()
        
        if (on):            
            paleta.setColor(self.label_9.backgroundRole(), QColor.fromRgbF(1, 0, 0, alpha=1))
        else:
            paleta.setColor(self.label_9.backgroundRole(), QColor.fromRgbF(0, 1, 0, alpha=1))
        self.label_9.setPalette(paleta)
    
    def __init__(self):
        super(mainWidget, self).__init__()
        
        uic.loadUi('iRobotGUI.ui',self)
#         self.robot = iRobot(self, "/dev/rfcomm0")
#         self.dial.setValue(500)
        self.label_3.setPixmap(QPixmap("./irobotcreate.jpg"))

        self.__task1 = iRobotTask1(self.robot)
        self.__task2 = iRobotTask2(self.robot)
        self.__task3 = iRobotTask3(self.robot)
        self.__task4 = iRobotTask4(self.robot)
        self.__task5 = iRobotTask5(self.robot)
        self.__task6 = iRobotTask6(self.robot)

        layout = self.layout()
        self.arrowButton = CMButton("Press Me to start Manual Control")
        layout.addWidget(self.arrowButton)
        self.arrowButton.clicked.connect(self.slotTask6Go)
        
        self.show()
        
        self.trigger.connect(self.guiUpdate)
        
        
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
        
    def slotStreamStart(self):
        self.robot.streamStart() # initialize timer for request more data
        self.__threadComm = threading.Thread(target=self.robot.streamRead) # initialize reading stream
        self.__threadComm.start()
        
#     def slotTask1Stop(self):
#         self.robot.streamStop()
        
    def slotTask1Go(self):
        self.__timerLastTime = time.time()
        self.robot.positioning.reinit()
        self.__task1.reset()
        self.__currentTask = 1
        
    def slotTask2Go(self):
        self.__timerLastTime = time.time()
        self.robot.positioning.reinit()
        self.__task2.reset()
        self.__currentTask = 2
        
    def slotTask3Go(self):
        self.__timerLastTime = time.time()
#         self.robot.positioning.reinit()
        self.__task3.reset()
        tx = int(self.task1LineEditTargetX.text())
        ty = int(self.task1LineEditTargetY.text())
        self.__task3.setTarget(tx, ty)
        self.__currentTask = 3
        
    def slotTask4Go(self):
        self.__timerLastTime = time.time()
        self.robot.positioning.reinit()
        self.__task4.reset()
        self.__currentTask = 4
    
    def slotTask5Go(self):
        self.__timerLastTime = time.time()
#         self.robot.positioning.reinit()
        self.__task5.reset()
        self.__currentTask = 5
    def slotTask6Go(self):
        self.__task6.reset()
        self.__currentTask = 6
        
    def slotSensorsChanged(self):
        print("iRobot Sensor data changed")
    

    
    def guiUpdate(self):
        self.rightWheelOvercurrent(self.robot.sensors.rightWheelOC())
        self.leftWheelOvercurrent(self.robot.sensors.leftWheelOC())
        
        self.dial.setValue(self.robot.actualRightWheelVel())
        self.dial_2.setValue(self.robot.actualLeftWheelVel())
        print(self.robot.sensors.wallSignal())
        self.progressBar.setValue(self.robot.sensors.wallSignal())
        self.progressBar_2.setValue(abs(self.robot.sensors.current()))
        self.lineEdit.setText(Qt.QString.number(self.robot.sensors.voltage()))
        self.lineEdit_2.setText(Qt.QString("%1/%2").arg(self.robot.sensors.charge()).arg(self.robot.sensors.capacity()))
        self.lineEdit_3.setText(Qt.QString.number(self.robot.sensors.temperature()))
    def taskUpdate(self):
        global shuttingDown
        # TODO:  Get angle and distance
        print('task regulate')
        
        self.trigger.emit()

        
        
        if (self.__currentTask == 1):
            print('task regulate 1')
            self.__task1.update()
        elif (self.__currentTask == 2):
            print('task regulate 2')
            self.__task2.update()
        elif (self.__currentTask == 3):
            print('task regulate 3')
            self.__task3.update()            
        elif (self.__currentTask == 4):
            print('task regulate 4')
            self.__task4.update()  
        elif (self.__currentTask == 5):
            print('task regulate 5')
            self.__task5.update()  
        elif (self.__currentTask == 6):
            print('task regulate 6')
            self.__task6.setKeyState(self.arrowButton.keyState())
            self.__task6.update()  
            
app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

