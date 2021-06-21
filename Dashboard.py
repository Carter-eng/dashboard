#dashboard window for lab robots
#runs as a server to ensure proper connectivity between robots
import sys
import socket
import selectors
import types
from  PyQt5.Qtcore import Qt,
from PyQt5.QtGui import QPainter,
from PyQt5.QtWidgets import QWidget, QMainWindow

class DashBoard(QMainWindow):
    def __init__(self, parent = None):
        super(DashBoard, self).__init__(parent)
        self.setGeometry(0,0,1200,1030)
        



if __name__ == '__main__':
        import sys

        app = QApplication(sys.argv)
        window = DashBoard()
        window.show()
        sys.exit(app.exec_())
