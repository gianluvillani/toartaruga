#!/usr/bin/env python

import rospy
import numpy as np
import math
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from low_level_interface.msg import lli_ctrl_request


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

class obstacle_measurement:
	def __init__(self):
		self.obstacle_msg = Obstacles()
		self.segment_obstacles = []		
		self.circle_obstacles = []
		self.ctrl_vel = 0
		self.ctrl_ang = 0

		self.min_dist = 0.5

		# Access rosparams
		self.obstacles_top = rospy.get_param(rospy.get_name() + '/obstacles_topic')
		self.danger_top = rospy.get_param(rospy.get_name() + '/danger_topic')	
		

		# Publishers and Subscribers
		self.pub_danger = rospy.Publisher(self.danger_top, Float32)
		self.sub_obstacles = rospy.Subscriber(self.obstacles_top, Obstacles, self.save_obstacles)
		self.sub_control = rospy.Subscriber("/lli/lli_ctrl_request", lli_ctrl_request, self.save_ctrl_state)

	def save_ctrl_state(self, ctrl_msg):
		self.ctrl_vel = ctrl_msg.velocity
		self.ctrl_ang = ctrl_msg.steering
			

	def save_obstacles(self, obstacle_msg):
		self.obstacle_msg = obstacle_msg

	def parse_obstacles(self, obstacle_msg):
		self.segment_obstacles = []	
		self.circle_obstacles = []	
		for segment in obstacle_msg.segments:
			self.segment_obstacles.append([(segment.first_point.x, segment.first_point.y), (segment.last_point.x, segment.last_point.y)])
		for circle in obstacle_msg.circles:
			self.circle_obstacles.append((circle.center.x , circle.center.y, circle.radius))	
			
		
	def circle_collision(self, circle_obstacle): # A circle_obstacle is a tuple (x_coordinate, y_coordinate, radius)
		x_obs = circle_obstacle[0]
		y_obs = circle_obstacle[1]
		r_obs = circle_obstacle[2]
		
		rho_obs, phi_obs = cart2pol(x_obs, y_obs)
		rotation_angle = self.ctrl_ang/100*math.pi/4		
		
		phi_obs += rotation_angle
		
		x_obs, y_obs = pol2cart(rho_obs, phi_obs)
		
		r_car = 0.25
		# Adaptive minimum allowed distance
		min_dist = self.min_dist + 0.1*abs(self.ctrl_vel)
		if x_obs < - math.fabs(0.5*y_obs) and math.fabs(y_obs) <  r_car + r_obs + 0.2 and math.sqrt(x_obs**2 + y_obs**2) < r_car + min_dist:
			#print(True)
			return True
		else:
			False
	def publish_danger(self):
		danger_msg = Float32()
		obstacle_found = False
		for circle in self.circle_obstacles:
			if self.circle_collision(circle):
				obstacle_found = True
		if obstacle_found:
			danger_msg.data = 1
		else:		
			danger_msg.data = 0
		self.pub_danger.publish(danger_msg)

		

	def segment_distance(self, segment_obstacle): # A segment_obstacle is a list [(first_point_x, first_point_y), (last_point_x, last_point_y)]
		x_obs = circle_obstacle[0]
		y_obs = circle_obstacle[1]
		r_obs = circle_obstacle[2]
		r_car = 0.25
		if x_obs < - math.fabs(0.5*y_obs) and math.fabs(y_obs) <  r_car + r_obs + 0.1:
			return math.sqrt(x_obs**2 + y_obs**2)
		

if __name__ == "__main__":
	rospy.init_node('obstacle_measurement', anonymous=True)
	rate = rospy.Rate(10)
	my_obstacle_measurement = obstacle_measurement()
	while not rospy.is_shutdown():
		my_obstacle_measurement.parse_obstacles(my_obstacle_measurement.obstacle_msg)
		my_obstacle_measurement.publish_danger()
		rate.sleep()
	

