import numpy as np
from time import sleep
import subprocess
import threading
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg._PoseStamped import PoseStamped


class Dashboard_pinger(robotAddresses,robotNumber,robotNames):
	def __init__(self):
		self.ping = ''
		self.ping_time = subprocess.Popen(["/bin/ping", "-c1", "-w100", str(robotAddresses[robotNumber])], stdout=subprocess.PIPE).stdout.read()
		self.ping = str(self.ping_time)
		threading.Thread(target=self.pinger, args=(robotNumber,)).start()		
		
	def pinger(self,robotNumber):
		pingStr = self.robotNames[robotNumber] + "PingTime"
        pingPub = rospy.Publisher(pingStr, Float64, queue_size=20)
        while True:
            ping_response = subprocess.Popen(["/bin/ping", "-c1", "-w100", str(self.robotAddresses[robotNumber])], stdout=subprocess.PIPE).stdout.read()
            self.ping = str(ping_response)
            pingList = self.ping.split(' ')
            if self.ping.find('Destination')==-1:
                pingTimeStr = pingList[12].split('=')
                pingFloat = float(pingTimeStr[1])
                pingPub.publish(pingFloat)
            else:
                pingPub.publish(3000)
            sleep(0.4)

class Dashboard_Pose(robotAddresses,robotNumber,robotNames):
	def __init__(self):
		self.pose = ''
		threading.Thread(target=self.subscriberThread, args=(robotNames,robotNumber,)).start()

	def subscriberThread(self,robotNames,robotNumber):
        topicName = "/"+str(robotNames[robotNumber])+"/pose"
        rospy.Subscriber(topicName,PoseStamped,self.poseUpdate,(robotNumber))
        rospy.spin()

    def poseUpdate(self,msg,args):
        position = msg.pose.position
        self.pose = "X:%f, Y:%f, Z:%F"%(position.x, position.y, position.z)
