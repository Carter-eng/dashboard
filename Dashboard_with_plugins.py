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
from dashboard_plugins import class_labels
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
        self.labels = []
        self.classes = []\
        self.class_inits =[]
        self.classes = self.get_classes(dashboard_plugins)
        for o in range(len(self.classes)):
            x = self.classes[o](0,0,0)
            self.labels.append(x.label)
        self.class_storage = [[] for i in range(len(self.classes))]
        self.listenerThread = threading.Thread(target=self.listener,)
        self.listenerThread.start()
        self.robotNames = []
        self.robotAddresses = []
        self.errorCounter = []
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
            threading.Thread(target=self.pinger, args=(robotNumber,)).start()
            self.errorCounter.append(0)
            for j in range(len(self.classes)):
                self.class_storage[j].append(self.classes[j](self.robotAddresses,robotNumber,self.robotNames))
            self.robotCounter += 1

    def pinger(self,robotNumber):
        pingStr = self.robotNames[robotNumber] + "PingTime"
        pingPub = rospy.Publisher(pingStr, Float64, queue_size=20)
        while True:
            ping_response = subprocess.Popen(["/bin/ping", "-c1", "-w100", str(self.robotAddresses[robotNumber])], stdout=subprocess.PIPE).stdout.read()
            self.ping[robotNumber] = str(ping_response)
            pingList = self.ping[robotNumber].split(' ')
            if self.ping[robotNumber].find('Destination')==-1:
                pingTimeStr = pingList[12].split('=')
                pingFloat = float(pingTimeStr[1])
                pingPub.publish(pingFloat)
            else:
                pingPub.publish(3000)
            sleep(0.4)


    def get_classes(self,module):
        class_list = inspect.getmembers(module, inspect.isclass)
        class_list_match = [
            cl for (nm, cl) in class_list if nm.startswith('Dashboard_')
        ]
        return class_list_match

    def getname(self):
        data = self.conn.recv(1024).decode()
        return(str(data))

    def generate_table(self) -> Table:
        table = Table(title="Robot information", border_style="yellow")
        table.add_column("NAME")
        table.add_column("IP")
        table.add_column("Ping Time")
        for k in range(len(self.labels)):
            table.add_column(self.labels[k])

        for i in range(self.robotCounter):
            if self.ping[i].find('Destination')==-1:
                if self.errorCounter == 101:
                    statusStr = self.robotNames[i] + " is back online"
                    self.statusPub.publish(statusStr)
                t = self.ping[i].split(' ')
                column_vals = ("[cyan]"+self.robotNames[i])+"[/cyan]","[magenta]" +str(self.robotAddresses[i])+"[/magenta]","[red]"+t[12]+ " ms" +"[/red]")
                for l in range(len(self.labels)):
                    column_vals = column_vals + ("[green]"+self.class_storage[i][l].value+"[green]")
                self.errorCounter[i] = 0
                table.add_row(str(*column_vals)
            elif self.errorCounter[i] < 100:
                column_vals = (str(self.robotNames[i]),str(self.robotAddresses[i]),"Robot Not Found")
                for p in range(len(self.labels)):
                    column_vals = column_vals + ("No Data")
                table.add_row(str(self.robotNames[i]),str(self.robotAddresses[i]),"Robot Not Found", "X: 0, Y:0, Z:0","X: 0, Y:0, Z:0")
                self.errorCounter[i] +=1
            elif self.errorCounter[i] == 100:
                statusStr = self.robotNames[i] + " is no longer online"
                self.statusPub.publish(statusStr)
                self.errorCounter[i] +=1
            else:
                pass
        return table

x = RobotDashboard()
with Live(
    Panel(x.generate_table(), title="Robots", border_style="blue"),
    refresh_per_second=2,
) as live:
    while True:
        sleep(0.4)

        live.update(Panel(x.generate_table(),title = "Robots", border_style="blue"))
