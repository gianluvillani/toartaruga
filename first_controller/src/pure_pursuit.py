#! /usr/bin/env python

#import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from low_level_interface.msg import lli_ctrl_request
from tf.transformations import euler_from_quaternion 
from control import controller

class pure_pursuit(controller):
	
	def __init__(self, v = 1, k = 0.1, lf = 1, l = 2):
		"""
		lf is the look-ahead distance.
		v is the velocity.
		k is a parameter that regulates how much the look-ahead distance changes with the speed.
		"""
		self.lf = lf
		self.l = l
		self.v = v
		self.k = k
		self.state_available = False
		self.path_available = False

		#Publishers/Subscribers
		self.pub_steer_control = rospy.Publisher('/lli/ctrl_request', lli_ctrl_request)
		self.sub_odom = rospy.Subscriber('/simulator/odom', Odometry, self.parse_state)
		self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.parse_path)

	def parse_state(self, odom_msg):
		"""
		parse_state saves the state variables from odom message. 
		"""
		self.x = odom_msg.pose.pose.position.x
		self.y = odom_msg.pose.pose.position.y
		orientation_q = odom_msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		self.yaw = euler_from_quaternion(orientation_list)[2]
		self.state_available = True

	def parse_path(self, path_msg):
		for pose in path_msg.poses:
			self.cx = pose.pose.position.x
			self.cy = pose.pose.position.y
		self.path_available = True

	def publish_control(self):
		linear_vel = self.v
		angular_vel = self.compute_control_signal()*(400/math.pi)
		ctrl_request_msg = lli_ctrl_request()
		ctrl_request_msg.velocity = linear_vel
		ctrl_request_msg.steering = angular_vel
		pub.publish(ctrl_request_msg)	

	def calc_target_index(self):

	    # search nearest point index
	    dx = [self.x - icx for icx in self.cx]
	    dy = [self.y - icy for icy in self.cy]
	    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
	    ind = d.index(min(d))
	    L = 0.0
	    Lf = self.k * self.v + self.lf

	    # search look ahead target point index
	    while Lf > L and (ind + 1) < len(self.cx):
	        dx = self.cx[ind + 1] - self.cx[ind]
	        dy = self.cx[ind + 1] - self.cx[ind]
	        L += math.sqrt(dx ** 2 + dy ** 2)
	        ind += 1 
	    return ind

	def compute_control_signal(self):

	    ind = self.calc_target_index()

	    if ind < len(self.cx):
	        tx = self.cx[ind]
	        ty = self.cy[ind]
	    else:
	        tx = self.cx[-1]
	        ty = self.cy[-1]
	        ind = len(self.cx) - 1

	    alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw

	    if self.v < 0:  # back
	        alpha = math.pi - alpha

	    Lf = self.k * self.v + self.lf

	    delta = math.atan2(2.0 * self.l * math.sin(alpha) / self.lf, 1.0)

	    return delta


if __name__ == "__main__":
    rospy.init_node('controller_node', anonymous=True)
	rate = rospy.Rate(20)
	my_controller = pure_pursuit(l=0.35, lf = 0.20)
	while not rospy.is_shutdown():
		if my_controller.state_available and my_controller.state_available:
			self.publish_control()
			rate.sleep()





	







		
