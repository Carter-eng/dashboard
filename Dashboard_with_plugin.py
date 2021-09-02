"""
Dashboard for experimental use in the
Boston University Robotics Laboratory.
Dashboard dislays Host Name, IP Address,
Ping Time, Local Pose Estimate, and 
Optitrack Pose Estimate.
Created using Python Rich Library
found here: https://github.com/willmcgugan/rich
For use with the Dashboard_Client.py script
found onboard the lab robots.
Created by Carter Berlind, 2021 
"""
from time import sleep
from rich.columns import Columns
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
import socket
from typing import OrderedDict
import dashboard_plugins
import inspect
import subprocess
import threading
import numpy as np
import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg import String, Float64

#Class storing the table definition and associated variables and functions
class RobotDashboard:
    def __init__(self):
        rospy.init_node("multirobot_dashboard",anonymous=True)
        self.statusPub = rospy.Publisher('robotStatus', String, queue_size=20)
        self.HOST = '192.168.1.170'
        self.PORT = 40006
        self.server = socket.socket()
        self.server.bind((self.HOST,self.PORT))
        self.server.listen()
        self.robotCounter = 0
        self.ping = []
        self.get_classes(dashboard_plugins)
        self.listenerThread = threading.Thread(target=self.listener,)
        self.listenerThread.start()
        self.robotNames = []
        self.robotAddresses = []
        self.errorCounter = []
        self.poses = []
        self.pingPubs = []
        
#Thread that listens for Robots to connect and registers their information
    def listener(self):
        while True:
            self.conn, self.address = self.server.accept()
            CurrentName = self.getname()
            statusStr = CurrentName + " is online"
            self.statusPub.pubblish(statusStr)
            self.robotNames.append(CurrentName)
            self.robotAddresses.append(self.address[0])
            self.conn.close()
            robotNumber = self.robotCounter
            ping_response = subprocess.Popen(["/bin/ping", "-c1", "-w100", str(self.address[0])], stdout=subprocess.PIPE).stdout.read()
            self.ping.append(str(ping_response))
            self.poses.append('')
            threading.Thread(target=self.pinger, args=(robotNumber,)).start()
            threading.Thread(target=self.subscriberThread, args=(CurrentName,robotNumber,)).start()
            self.errorCounter.append(0)
            self.robotCounter += 1

    def get_classes(self,module):
        class_list = inspect.getmembers(module, inspect.isclass)
        class_list_match = [
            cl for (nm, cl) in class_list if nm.startswith('Dashboard_')
        ]
        return class_list_match

    def getname(self):

        data = self.conn.recv(1024).decode()


        return(str(data))