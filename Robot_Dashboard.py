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
            
#Thread for each robot that individually pings them
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
            
#Subscribes to the topics wherever necessary
   def subscriberThread(self,CurrentName,robotNumber):
        topicName = "/"+str(CurrentName)+"/pose"
        rospy.Subscriber(topicName,PoseStamped,self.poseUpdate,(robotNumber))
        rospy.spin()
        
#Pose callback function
    def poseUpdate(self,msg,args):
        position = msg.pose.position
        self.poses[args] = "X:%f, Y:%f, Z:%F"%(position.x, position.y, position.z)
                
#creates the table that displays the information from the robots
    def generate_table(self) -> Table:
        table = Table(title="Robot information", border_style="yellow")
        table.add_column("NAME")
        table.add_column("IP")
        table.add_column("PING")
        table.add_column("POSE OPTITRACK")
        table.add_column("POSE LOCAL ESTIMEATE")
        for i in range(self.robotCounter):
            if self.ping[i].find('Destination')==-1:
                if self.errorCounter == 101:
                    statusStr = self.robotNames[i] + " is back online"
                    self.statusPub.publish(statusStr)
                self.errorCounter[i] = 0
                t = self.ping[i].split(' ')
                table.add_row(str("[cyan]"+self.robotNames[i])+"[/cyan]","[magenta]" +str(self.robotAddresses[i])+"[/magenta]","[red]"+t[12]+ " ms" +"[/red]","[green]"+self.poses[i]+"[/green]","[blue]X:0, Y:0, Z:0[/blue]")
            elif self.errorCounter[i] < 100:
                table.add_row(str(self.robotNames[i]),str(self.robotAddresses[i]),"Robot Not Found", "X: 0, Y:0, Z:0","X: 0, Y:0, Z:0")
                self.errorCounter[i] +=1
            elif self.errorCounter[i] == 100:
                statusStr = self.robotNames[i] + " is no longer online"
                self.statusPub.publish(statusStr)
                self.errorCounter +=1
            else:
                pass
        return table
        
#recieves the hostname from the robot when they run the client
    def getname(self):

        data = self.conn.recv(1024).decode()


        return(str(data))

#Generates the display and updates it
x = RobotDashboard()
with Live(
    Panel(x.generate_table(), title="Robots", border_style="blue"),
    refresh_per_second=2,
) as live:
    while True:
        sleep(0.4)

        live.update(Panel(x.generate_table(),title = "Robots", border_style="blue"))
        
       
