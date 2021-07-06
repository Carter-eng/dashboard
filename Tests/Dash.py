#dashboard window for lab robots
#runs as a server to ensure proper connectivity between robots
import sys
import socket
import selectors
import types
import time
from pythonping import ping
from  PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import QWidget, QMainWindow, QApplication, QLabel,QPushButton

class DashBoard(QMainWindow):
    def __init__(self, parent = None):
        super(DashBoard, self).__init__(parent)
        self.setGeometry(0,0,1200,500)
        self.pingTime = ping('10.192.239.68')       
        self.pingList = QLabel("Spot Ping is "+str(self.pingTime.rtt_avg_ms) + " ms",self)
        self.pingList.move(10,10)
        self.pingList.resize(200,20)
        self.pingList.show()
        self.refresh = QPushButton('REFRESH',self)
        self.refresh.move(10,30)
        self.refresh.clicked.connect(self.refreshFunction)
        
    def refreshFunction(self):
        self.pingList.clear()
        self.pingTime = ping('10.192.239.68')       
        self.pingList = QLabel("Spot Ping is "+str(self.pingTime.rtt_avg_ms) + " ms",self)
        self.pingList.move(10,10)
        self.pingList.resize(200,20)
        self.pingList.show()



if __name__ == '__main__':
        import sys

        app = QApplication(sys.argv)
        window = DashBoard()
        window.show()
        sys.exit(app.exec_())
