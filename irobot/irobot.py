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
from time import sleep  
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import QObject, pyqtSignal


class iRobotTask1:
    __robot = 0
    __currDistance = 0
    __currAngle = 0
    def __init__(self, robot):
        self.__robot = robot
    def update(self):
        self.positionCalculate()
        if (self.__robot.__currDistance >= 1000):
            self.__robot.stop()
        print ('update')
    def positionCalculate(self):
        self.__currDistance += self.__robot.sensors.distance()
        self.__currAngle += self.__robot.sensors.angle()


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
                    self.__distance = -((ord(data[i])&0x7f)*256 + ord(data[i+1]))
                else:
                    self.__distance = (ord(data[i])&0x7f)*256 + ord(data[i+1])
                print("Distance: " + str(self.__distance))
#                 self.__distance = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 20:
                if ord(data[i])&0x80:
                    self.__angle = -((ord(data[i])&0x7f)*256 + ord(data[i+1]))
                else:
                    self.__angle = (ord(data[i])&0x7f)*256 + ord(data[i+1])
                print("Angle: " + str(self.__angle))
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
    __streamRunning = 0
    __packetLengthSemaphore = 0
    sensors = 0
    __currAngle = 0
    __currDistance = 0
    __task1 = 0
    
    def __init__(self, path):
        self.sensors = iRobotSensors()
#       super(iRobot, self).__init__()
        self.__port = serial.Serial(path, baudrate=57600)
        self.__port.open()
        self.__packetLengthSemaphore = threading.Semaphore()
        self.__task1 = iRobotTask1(self)

    def modeSafe(self):
        self.__port.write("\x80\x83")
#         self.__port.write(str([0x80, 0x83]))
    def modeFull(self):
        self.__port.write("\x80\x84")
    def goCircle(self):
        self.__port.write("\x89\x00\xff\x00\xff")
    def goStraight(self):
        self.__port.write("\x91\x00\xff\x00\xff")
    def stop(self):
        self.__port.write("\x91\x00\x00\x00\x00")
        self.__port.write("\x80")
 
    def streamStart(self, requestedPacketsString):
        self.goStraight()
        self.__port.write(requestedPacketsString)
#         self.__port.write("\x94\x04\x07\x1B\x16\x17")
    def streamRead(self):
        self.__streamRunning = True
        while self.__streamRunning:
            if self.__packetLength == 0:    # get synchronized
                sync = ord(self.__port.read(1))
                if sync == 19:
                    self.__packetLengthSemaphore.acquire()
                    self.__packetLength = ord(self.__port.read(1))
                    if (self.__packetLength != 0):
                        trash = self.__port.read(self.__packetLength+1)
                    self.__packetLengthSemaphore.release()
            elif self.__packetLength > 0:
                sync = ord(self.__port.read(1))
                if sync == 19:
                    self.__packetLengthSemaphore.acquire()
                    data = self.__port.read(self.__packetLength+2)
                    self.__packetLengthSemaphore.release()
                    # checksum calculations START
#                     sum = 0
#                     for i in range(0, len(data)):
#                         sum += ord(data[i])
#                     if sum&(0xff) != 0:     # bad checksum
#                         self.__packetLength = 0
#                         continue
                    # checksum calculations END
                    self.sensors.parseStream(data)
                    self.__task1.update()
                else :
                    self.__packetLength = 0
        print("iRobot Stream read thread Stopped")
    def streamStop(self):
        self.__port.write("\x96\x00")
        self.__streamRunning = False


    def __del__(self):
        self.streamStop()
        self.stop()
        self.__port.close()
     
class mainWidget(QtGui.QWidget):
    __robot = 0
    
    def __init__(self):
        super(mainWidget, self).__init__()
        uic.loadUi('iRobotGUI.ui',self)
        self.__robot = iRobot("/dev/rfcomm1")
#         self.__robot.sensors.valueChanged.connect(self.slotSensorsChanged)
        self.show()
        
    def slotStart(self):
        self.__robot.modeFull()
        print("iRobot Control Enabled")
    def slotStop(self):
        self.__robot.streamStop()
        self.__robot.stop()
        print("iRobot Control Disabled")
        
    def slotTask1Start(self):
        
        
        self.__robot.streamStart("\x94\x06\x07\x1B\x16\x17\x13\x14")    # bumps, wallSignal, voltage, current, distance, angle
        self.__threadComm = threading.Thread(target=self.__robot.streamRead)
        self.__threadComm.start()
    def slotTask1Stop(self):
        self.__robot.streamStop()
    def slotSensorsChanged(self):
        print("iRobot Sensor data changed")
             
             
app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

