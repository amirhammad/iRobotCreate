from PyQt4 import QtCore, QtGui, uic
import sys

class mainWidget(QtGui.QWidget):
#     
    def __init__(self):
        super(mainWidget, self).__init__()
        uic.loadUi('untitled.ui',self)
        self.show()
    def slotStart(self):
        print('start')
    def slotStop(self):
        print('stop')

        
app = QtGui.QApplication(sys.argv)
w = mainWidget()
app.exec_()