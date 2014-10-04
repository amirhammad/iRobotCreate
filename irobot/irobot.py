""" Copyright Amir Hammad & Peter Belica  			"""
""" Published under GNU GENERAL PUBLIC LICENSE (see LICENSE)    """
import serial
import threading
import sys
from time import sleep  
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import QObject, pyqtSignal


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
                self.__distance = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 20:
                self.__angle = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 21:
                pass
            elif pid == 22:
                self.__voltage = ord(data[i])*256 + ord(data[i+1]) 
                # print("Voltage: " + str(self.__voltage)) 
                i += 1
            elif pid == 23:
                if ord(data[i])&(0x80) != 0:    # negative current
                    self.__current = -1*((ord(data[i])&(0x7f))*256 + ord(data[i+1]))
                else:   # positive current
                    self.__current = ord(data[i])&(0x7f)*256 + ord(data[i+1])
                # print("Current: " + str(self.__current)) 
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
    
    def __init__(self, path):
        self.sensors = iRobotSensors()
#       super(iRobot, self).__init__()
        self.__port = serial.Serial(path, baudrate=57600)
        self.__port.open()
        self.__packetLengthSemaphore = threading.Semaphore()

    def modeSafe(self):
        self.__port.write("\x80\x83")
    def stop(self):
        self.__port.write("\x91\x00\x00\x00\x00")
        self.__port.write("\x80")
 
    def streamStart(self, requestedPacketsString):
        self.__port.write(requestedPacketsString)
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
                    sum = 0
                    for i in range(0, len(data)):
                        sum += ord(data[i])
                    if sum&(0xff) != 0:     # bad checksum
                        self.__packetLength = 0
                        continue
                    # checksum calculations END
                    self.sensors.parseStream(data)
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
        self.__robot = iRobot("/dev/rfcomm11")
#         self.__robot.sensors.valueChanged.connect(self.slotSensorsChanged)
        self.show()
        
    def slotStart(self):
        self.__robot.modeSafe()
        print("iRobot Control Enabled")
    def slotStop(self):
        self.__robot.streamStop()
        self.__robot.stop()
        print("iRobot Control Disabled")
        
    def slotTask1Start(self):
        self.streamStart("\x94\x04\x07\x1B\x16\x17")    # bumps, wallSignal, voltage, current
        self.__threadComm = threading.Thread(target=self.__robot.streamRead)
        self.__threadComm.start()
    def slotTask1Stop(self):
        self.__robot.streamStop()
    def slotSensorsChanged(self):
        print("iRobot Sensor data changed")
             
             
app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

