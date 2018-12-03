#! /usr/bin/env python

import rospy
import math
import numpy as np
from std_srvs.srv import Empty
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from low_level_interface.msg import lli_ctrl_request
from tf.transformations import euler_from_quaternion 
from controller import controller
from std_msgs.msg import Float32, Bool

class pid_controller(controller):
	
	def __init__(self, K_yaw_P = 1 , K_yaw_D = 0, K_yaw_I = 0, d_des = 1, K_dis_P = 1, K_dis_D = 0, K_dis_I = 0, Ts = 0.0125):
		self.start_steering = 0
		self.Ts = Ts
		self.K_yaw_P = K_yaw_P
		self.K_yaw_D = K_yaw_D
		self.K_yaw_I = K_yaw_I
		self.K_dis_P = K_dis_P
		self.K_dis_D = K_dis_D
		self.K_dis_I = K_dis_I
		self.d_des = d_des
		self.error_I_yaw = 0
		self.error_I_dis = 0
		self.error_yaw = 0
		self.error_dis = 0
		self.old_error_yaw = 0
		self.old_error_dis = 0
		self.state_available = False
		self.waypoint_available = False
		self.path_available = False
		self.TRACKING = True

		# Access rosparams
		self.steer_control_top = rospy.get_param(rospy.get_name() + "/steer_control_topic")
		self.car_pose_top = rospy.get_param(rospy.get_name() + "/car_pose_topic")
		self.waypoint_top = rospy.get_param(rospy.get_name() + "/waypoint_topic")
		self.path_top = rospy.get_param(rospy.get_name() + "/leader_path_topic")
		self.command_controller_follow_top = rospy.get_param(rospy.get_name() + "/command_controller_follow_topic")

		# Publishers/Subscriber
		self.pub_steer_control = rospy.Publisher(self.steer_control_top, lli_ctrl_request)
		#self.sub_pose = rospy.Subscriber('/simulator/odom', Odometry, self.save_state)
		self.sub_pose = rospy.Subscriber(self.car_pose_top, PoseStamped, self.save_state)
		self.sub_waypoint = rospy.Subscriber(self.waypoint_top, PoseStamped, self.save_waypoint)
		self.sub_path = rospy.Subscriber(self.path_top, Path, self.save_path)
		self.sub_start_stop_follow_controller = rospy.Subscriber(self.command_controller_follow_top, Bool, self.start_stop)
		
	def start_stop(self, start_stop_msg):
		self.TRACKING = start_stop_msg.data
	
	#def update_danger(self, danger_msg):
	#	if danger_msg.data > 0.5:
	#		self.TRACKING = False
	#	else:
	#		self.TRACKING = True
		
	def parse_state(self, state_msg):
		#print "STATE RECEIVED!"
                #self.x = odom_msg.pose.pose.position.x
                #self.y = odom_msg.pose.pose.position.y
		self.x = state_msg.pose.position.x
                self.y = state_msg.pose.position.y
                #orientation_q = odom_msg.pose.pose.orientation
		orientation_q = state_msg.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                self.yaw = euler_from_quaternion(orientation_list)[2]
                self.state_available = True
	
        
	def save_state(self, state_msg):
		self.state = state_msg
		self.state_available = True
        
	def save_waypoint(self, waypoint_msg):
		self.waypoint = waypoint_msg
		self.waypoint_available = True

	def save_path(self, path_msg):
		self.path = path_msg
		self.path_available = True

	def parse_path(self, path_msg):			
		self.x_path = []
		self.y_path = []
		for pose in path_msg.poses:
			self.x_path.append(pose.pose.position.x)
			self.y_path.append(pose.pose.position.y)

	def parse_waypoint(self, waypoint_msg):
		self.x_w = waypoint_msg.pose.position.x
                self.y_w = waypoint_msg.pose.position.y
		orientation_q = waypoint_msg.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                self.yaw_w = euler_from_quaternion(orientation_list)[2]
		#self.x_w = 0
		#self.y_w = 0

	def publish_control(self, steering_degree, velocity):
		linear_control = velocity	#check the map to vel
		angular_control = steering_degree*(400/math.pi)*0.2+0.8*self.start_steering
		self.start_steering = angular_control
		ctrl_request_msg = lli_ctrl_request()
		ctrl_request_msg.velocity = int(linear_control)
		ctrl_request_msg.steering = int(angular_control)
		self.pub_steer_control.publish(ctrl_request_msg)
		#self.save_data(target_ind, steering_degree, linear_control)	
	"""
	def save_data(self, index, delta, velocity):
		out_file = open('/home/nvidia/catkin_ws/src/first_controller/data_figure_eight_l_35_f_95.txt',"a")	
		out_file.write(str(index)+', '+ str(delta)+', '+ str(velocity)+', '+ str(rospy.get_time()-self.starting_time)+ '\n' )
		out_file.close()
	"""

	def update_error_I(self):
		self.error_I_yaw += self.error_yaw*self.Ts
		self.error_I_dis += self.error_dis*self.Ts

	def derivative(self):
		error_D_dis = (self.error_dis - self.old_error_dis)/self.Ts
		error_D_yaw = (self.error_yaw - self.old_error_yaw)/self.Ts
		return error_D_yaw, error_D_dis

	def compute_velocity_angular(self):
		d = math.sqrt((self.x-self.x_w)**2 + (self.y-self.y_w)**2)
		self.old_error_dis = self.error_dis
		self.error_dis = d - self.d_des
               

		if abs(self.error_dis) < 0.1:
			self.error_dis = 0.0
			self.error_I_dis = 0.0

		yaw_des = math.atan2(self.y_w-self.y, self.x_w-self.x)
		self.old_error_yaw = self.error_yaw
		self.error_yaw = yaw_des - self.yaw
		#rospy.logerr("e_yaw+ = %s ,  e_yaw- = %s", self.error_yaw, self.error_yaw-math.pi*2)
		self.update_error_I()
	
		error_D_yaw, error_D_dis = self.derivative()

		v = self.K_dis_P*self.error_dis + self.K_dis_D*error_D_dis + self.K_dis_I*self.error_I_dis
		gamma = 3
		if v < 0:
			v = -(v-18)*(-1+math.exp(gamma*v))
			#v = v - 15
		if v > 0:
			v = (v+13)*(1-math.exp(-gamma*v))
			#v = v + 13

		if v > 100:
			v = 100

		if v < -100:
			v = -100

		delta = self.K_yaw_P*self.error_yaw + self.K_yaw_D*error_D_yaw + self.K_yaw_I*self.error_I_yaw
		if delta > math.pi/4:
			delta = math.pi/4
		if delta < -math.pi/4:
			delta = -math.pi/4
		if self.error_yaw > math.pi: 
			delta = -self.K_yaw_P*(2*math.pi-self.error_yaw) + self.K_yaw_D*error_D_yaw + self.K_yaw_I*self.error_I_yaw
			if delta > math.pi/4:
				delta = math.pi/4
			if delta < -math.pi/4:
				delta = -math.pi/4

		if self.error_yaw < -math.pi: 
			delta = self.K_yaw_P*(2*math.pi+self.error_yaw) + self.K_yaw_D*error_D_yaw + self.K_yaw_I*self.error_I_yaw
			if delta > math.pi/4:
				delta = math.pi/4
			if delta < -math.pi/4:
				delta = -math.pi/4
		return delta, v

	def compute_delta(self):

	
	    alpha = math.atan2(self.y_w - self.y, self.x_w - self.x) - self.yaw

	    #Lf = self.k * self.v + self.lf
	    Lf = self.error_dis + self.d_des
	    l = 0.5
	    
	    delta_pp = math.atan2(2.0 * l * math.sin(alpha) / Lf, 1.0)
	    if delta_pp > math.pi/4:
		delta_pp = math.pi/4
	    if delta_pp < -math.pi/4:
		delta_pp = -math.pi/4

	    return delta_pp


if __name__ == "__main__":
	rospy.init_node('pid_controller')
	my_controller = pid_controller(K_yaw_P = 1.2, K_yaw_D = 0.2, K_yaw_I = 0.01, K_dis_P = 15, K_dis_I = 1, K_dis_D = 2, d_des = 0.5)
	rate = rospy.Rate(80)
	#rospy.logerr("%s MAIN STARTED")
	while not rospy.is_shutdown():
		if my_controller.state_available and my_controller.path_available:
	  		my_controller.parse_path(my_controller.path)
	    		my_controller.parse_state(my_controller.state)
			if my_controller.TRACKING:
				delta, v = my_controller.compute_velocity_angular()
				delta_pp = my_controller.compute_delta()		
				#rospy.logerr(" speed = %s, error_dis = %s", v, my_controller.error_dis)		
				my_controller.publish_control(delta_pp, v)
			else:
				my_controller.publish_control(delta_pp, 0)
			rate.sleep()
