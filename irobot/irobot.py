import serial
import threading
from PyQt4 import QtCore, QtGui, uic
import sys
from gi.overrides.keysyms import Pause
from time import sleep  



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
    
    def parseStream(self, data):
        i=2
        while i < ord(data[1]):
#             lenData = getLenOfData(data[i])

            # PACKET ID
            pid = ord(data[i])
            
            # if you want to read more than one bytes, increment inside if
            i += 1 
            if pid == 7:
                self.__bumpRight = ord(data[i])&(0x01)
                self.__bumpLeft = ord(data[i])&(0x02)
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
                self.__voltage = ord(data[i])<<8 + ord(data[i+1]) 
                i += 1
            elif pid == 23:
                self.__current = ord(data[i])<<8 + ord(data[i+1])
                i += 1
            elif pid == 24:
                pass
            elif pid == 25:
                i += 1
            elif pid == 26:
                i += 1
            elif pid == 27:
                self.__wallSignal = ord(data[i])<<8 + ord(data[i+1]) 
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
                print('Wrong packet ID: ' + str(pid))
                
            i += 1
         
class iRobot():
    __port = 0
    __packetLength = 0
    __streamRunning = 0
    sensors = 0
    __packetLengthSemaphore = 0
    def __init__(self, path):
        self.sensors = iRobotSensors()
#         super(iRobot, self).__init__()
        __sem = threading.Semaphore()
#         self.__port = serial.Serial(path, baudrate=57600)
#         self.__port.open()
    
    def modeSafe(self):
        self.__port.write("\x80\x83")
        
    def go(self):
        self.__port.write("\x91\x01\xff\x01\xff")
    def stop(self):
        self.__port.write("\x91\x00\x00\x00\x00")
    def streamStart(self):
        self.__port.write("\x94\x01\x08")
    
    def streamRead(self):
        self.__streamRunning = True
        while self.__streamRunning:
            if self.__packetLength == 0:
                sync = ord(self.__port.read(1))
                if sync == 19:
                    self.__packetLengthSemaphore.acquire()
                    self.__packetLength = ord(self.__port.read(1))
                    if (self.__packetLength != 0):
                        self.__port.read(self.__packetLength - 1)
                    self.__packetLengthSemaphore.release()
            elif self.__packetLength > 0:
                sync = ord(self.__port_read(1))
                if sync == 19:
                    self.__packetLengthSemaphore.acquire()
                    data = ord(self.__port.read(self.__packetLength))
                    # TODO Calculate checksum
                    self.__packetLengthSemaphore.acquire()
                    self.sensors.parseStream(data)
                else :
                    self.__packetLength = 0
                
    def streamStop(self):
        self.__streamRunning = False
    def __del__(self):
#         self.__port.write("\x96\x00")
        self.__port.close()
        
class mainWidget(QtGui.QWidget):
    __robot = 0
    
    def __init__(self):
        super(mainWidget, self).__init__()
        uic.loadUi('untitled.ui',self)
        
        self.__robot = iRobot("/dev/rfcomm4")
        self.show()
    def slotStart(self):
#         self.__robot.streamRead()
        self.__threadComm = threading.Thread(target=self.__robot.streamRead)
        self.__threadComm.start()
    def slotStop(self):
        self.__robot.streamStop()
#         if self.__threadComm:
#             self.__threadComm.kill_received = True
        print('stop')
             
app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()

