import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Int32

rospy.init_node('controller')
pub=rospy.Publisher ('State', Int32)

class car:
	"""
	States = [x, y, yaw, v]
	Parameters = [dt, l, cx, cy]
	"""
	def __init__(self, x=0.0, y=0.0, yaw=0.0, v = 5, dt = 0.1, l = 7):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.dt = dt
		self.l = l
		self.cx, self.cy = self.generate_course()
		self.lf = 1

	def apply_control(self, a, delta):
		"""
		Simple kinematic model
		"""
		self.x = self.x + self.v*math.cos(self.yaw)*self.dt
		self.y = self.y + self.v*math.sin(self.yaw)*self.dt
		self.yaw = self.yaw + self.v/self.l*math.tan(delta)*self.dt
		self.v = self.v + a*self.dt

car1=car()
while not rospy.is_shutdown():
	pub.publish(



