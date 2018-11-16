#! /usr/bin/env python

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import time
from std_srvs.srv import Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from low_level_interface.msg import lli_ctrl_request
from tf.transformations import euler_from_quaternion 
from controller import controller
from std_msgs.msg import Float32, Bool

class pure_pursuit(controller):
	
	def __init__(self, v = 1, k = 0.0, lf = 1, l = 2):
		"""
		lf is the look-ahead distance.
		v is the velocity.
		k is a parameter that regulates how much the look-ahead distance changes with the speed.
		"""
		self.start_steering = 0
		self.lf = lf
		self.l = l
		self.v = v
		self.k = k
		self.state_available = False
		self.path_available = False
		self.cx = []
		self.cy = []
		self.starting_time=rospy.get_time()
		self.TRACKING = False
		self.last_nearest_index = 0
		self.counter = 0

		# Access rosparams
		self.steer_control_top = rospy.get_param(rospy.get_name() + "/steer_control_topic")
		self.car_pose_top = rospy.get_param(rospy.get_name() + "/car_pose_topic")
		self.path_top = rospy.get_param(rospy.get_name() + "/path_topic")
		self.command_controller_top = rospy.get_param(rospy.get_name() + "/command_controller_topic")

		# Publishers/Subscriber
		self.pub_steer_control = rospy.Publisher(self.steer_control_top, lli_ctrl_request)
		#self.sub_pose = rospy.Subscriber('/simulator/odom', Odometry, self.save_state)
		self.sub_pose = rospy.Subscriber(self.car_pose_top, PoseStamped, self.save_state)
		self.sub_path = rospy.Subscriber(self.path_top, Path, self.save_path)
		self.sub_start_stop_controller = rospy.Subscriber(self.command_controller_top, Bool, self.start_stop)
		
	def start_stop(self, start_stop_msg):
		self.TRACKING = start_stop_msg.data
	
	def update_danger(self, danger_msg):
		if danger_msg.data > 0.5:
			self.TRACKING = False
		else:
			self.TRACKING = True
		
	def parse_state(self, odom_msg):
		#print "STATE RECEIVED!"
                #self.x = odom_msg.pose.pose.position.x
                #self.y = odom_msg.pose.pose.position.y
		self.x = odom_msg.pose.position.x
                self.y = odom_msg.pose.position.y
                #orientation_q = odom_msg.pose.pose.orientation
		orientation_q = odom_msg.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                self.yaw = euler_from_quaternion(orientation_list)[2]
                self.state_available = True
        
	def save_state(self, odom_msg):
		self.odom = odom_msg
		self.state_available = True
        
	def save_path(self, path_msg):
		self.path = path_msg
		self.path_available = True

	def parse_path(self, path_msg):
		# TODO: This fix is just to debug pure_pursuit, find a solution
		#print "PATH PUBLISHED"				
		self.cx = []
		self.cy = []
		for pose in path_msg.poses:
			self.cx.append(pose.pose.position.x)
			self.cy.append(pose.pose.position.y)
		
	def publish_control(self, steering_degree, target_ind, velocity):
		linear_control = velocity	#check the map to vel
		angular_control = steering_degree*(400/math.pi)*0.2+0.8*self.start_steering
		self.start_steering = angular_control
		ctrl_request_msg = lli_ctrl_request()
		ctrl_request_msg.velocity = int(linear_control)
		ctrl_request_msg.steering = int(angular_control)
		self.pub_steer_control.publish(ctrl_request_msg)
		#self.save_data(target_ind, steering_degree, linear_control)	
	
	def save_data(self, index, delta, velocity):
		out_file = open('/home/nvidia/catkin_ws/src/first_controller/data_figure_eight_l_35_f_95.txt',"a")	
		out_file.write(str(index)+', '+ str(delta)+', '+ str(velocity)+', '+ str(rospy.get_time()-self.starting_time)+ '\n' )
		out_file.close()

	def calc_target_index(self):
	    
	    ind = None
	    # search nearest point index
	    if len(self.cx)!=0:
	    	dx = [self.x - icx for icx in self.cx]
	    	dy = [self.y - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

		

		if self.counter < 5:
			new_ind = d.index(min(d))
			self.counter += 1
			self.last_nearest_index
			print(self.last_nearest_index)
		else:
			new_d = d[self.last_nearest_index:self.last_nearest_index + len(self.cx)//5]
			
			extra = d[:len(self.cx)//5 - len(new_d)]
			new_d.extend(extra)
			new_ind = new_d.index(min(new_d))
			if len(extra) > 0:
				self.last_nearest_index = len(extra)-1
			else:
				self.last_nearest_index = new_ind

	    	ind = d.index(min(d))
		
		#ind = new_ind

		
	    	L = 0.0
	    	Lf = self.k * self.v + self.lf

	    	# search look ahead target point index
	    	while Lf > L and (ind + 1) < len(self.cx):
	        	dx = self.cx[ind + 1] - self.cx[ind]
	        	dy = self.cx[ind + 1] - self.cx[ind]
	        	L += math.sqrt(dx ** 2 + dy ** 2)
	        	ind +=1	
			if ind>=len(self.cx)-1:
				ind = 0
	    return ind

	def compute_delta(self,ind):

	    if ind ==None:
		return 0
	    tx = self.cx[ind]
	    ty = self.cy[ind]
	    alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw

	    if self.v < 0:  # back
	        alpha = math.pi - alpha

	    Lf = self.k * self.v + self.lf

	    delta = math.atan2(2.0 * self.l * math.sin(alpha) / Lf, 1.0)
	    if delta > math.pi/4:
		delta = math.pi/4
	    if delta < -math.pi/4:
		delta = -math.pi/4
	    return delta

	def compute_velocity(self, delta):
		return self.v


if __name__ == "__main__":
	rospy.init_node('Pure_pursuit_controller')
	rate = rospy.Rate(80)
	my_controller = pure_pursuit(l=0.2, lf = 0.35, v=20)
	print('MAIN STARTED')
	while not rospy.is_shutdown():
		if my_controller.state_available and my_controller.path_available:   
	  		my_controller.parse_path(my_controller.path)
	    		my_controller.parse_state(my_controller.odom)
			if my_controller.TRACKING:
		    		ind = my_controller.calc_target_index()
				delta = my_controller.compute_delta(ind)
				v = my_controller.compute_velocity(delta)
				my_controller.publish_control(delta, ind, v)
				print('HEEEERRREEEEE')
			else:
				my_controller.publish_control(0, 0, 0)
			rate.sleep()
