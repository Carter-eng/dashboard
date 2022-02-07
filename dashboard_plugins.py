import numpy as np
from time import sleep
import subprocess
import threading
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg._PoseStamped import PoseStamped





class Dashboard_Pose(robotAddresses,robotNumber,robotNames,robotType):
	def __init__(self):
		self.label = "Mocap Pose"
		self.value = ''
		if robotAddresses !=0:
			self.subscriber(robotNames,robotNumber)
		else:
			pass
	def subscriber(self,robotNames,robotNumber):
        topicName = "/"+str(robotNames[robotNumber])+"/pose"
        rospy.Subscriber(topicName,PoseStamped,self.poseUpdate,(robotNumber))
        rospy.spin()

    def poseUpdate(self,msg,args):
        position = msg.pose.position
        self.value = "X:%f, Y:%f, Z:%F"%(position.x, position.y, position.z)
