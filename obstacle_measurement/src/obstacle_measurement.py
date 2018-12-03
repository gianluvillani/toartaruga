#!/usr/bin/env python

import rospy
import numpy as np
import math
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import PoseStamped
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

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
		self.min_dist = 1
		self.ctrl_vel = 0
		self.ctrl_ang = 0
		self.scan_available = False

		# Access rosparams
		self.obstacles_top = rospy.get_param(rospy.get_name() + '/obstacles_topic')
		self.des_obstacle_top = rospy.get_param(rospy.get_name() + '/des_obstacle_topic')
		self.danger_top = rospy.get_param(rospy.get_name() + '/danger_topic')
		self.steer_control_top = rospy.get_param(rospy.get_name() + "/steer_control_topic")
		self.car_pose_top = rospy.get_param(rospy.get_name() + "/car_pose_topic")
		self.laser_scan_top = rospy.get_param(rospy.get_name() + "/laser_scan_topic")

		# Publishers and Subscribers
		self.pub_des_obstacle = rospy.Publisher(self.des_obstacle_top, PoseStamped)
		self.pub_danger = rospy.Publisher(self.danger_top, Float32)
		self.sub_obstacles = rospy.Subscriber(self.obstacles_top, Obstacles, self.save_obstacles)
		self.sub_control = rospy.Subscriber(self.steer_control_top, lli_ctrl_request, self.save_ctrl_state)
		self.sub_car_pose = rospy.Subscriber(self.car_pose_top, PoseStamped, self.save_state)
		self.sub_laser_scan = rospy.Subscriber(self.laser_scan_top, LaserScan, self.save_scan)

	def save_ctrl_state(self, ctrl_msg):
		self.ctrl_vel = ctrl_msg.velocity
		self.ctrl_ang = ctrl_msg.steering
			

	def save_obstacles(self, obstacle_msg):
		self.obstacle_msg = obstacle_msg
	
	def save_state(self, state_msg):
		self.state_msg = state_msg
	
	def save_scan(self, scan_msg):
		self.scan_msg = scan_msg
		self.scan_available = True

	def parse_obstacles(self, obstacle_msg):
		self.segment_obstacles = []	
		self.circle_obstacles = []	
		for segment in obstacle_msg.segments:
			self.segment_obstacles.append([(segment.first_point.x, segment.first_point.y), (segment.last_point.x, segment.last_point.y)])
		for circle in obstacle_msg.circles:
			self.circle_obstacles.append((circle.center.x , circle.center.y, circle.radius))	
			
	
	def parse_scan(self, scan_msg):
		self.obstacle_points = []
		if self.scan_available:
			range_list = scan_msg.ranges
			theta_min = scan_msg.angle_min
			angle_max = scan_msg.angle_max
			delta = scan_msg.angle_increment
			range_min = scan_msg.range_min
			range_max = scan_msg.range_max
			for i in range(len(range_list)):
				r = range_list[i]
				theta = theta_min + i*delta
				x = r*math.cos(theta)
				y = r*math.sin(theta)
				if range_min < r < range_max:
					self.obstacle_points.append((x,y))
	
		
	def parse_state(self, state_msg):
		self.x_car = state_msg.pose.position.x
                self.y_car = state_msg.pose.position.y
		orientation_q = state_msg.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                self.yaw_car = euler_from_quaternion(orientation_list)[2]
                self.state_available = True

	def circle_collision(self, circle_obstacle): # A circle_obstacle is a tuple (x_coordinate, y_coordinate, radius)
		x_obs = circle_obstacle[0]
		y_obs = circle_obstacle[1]
		r_obs = circle_obstacle[2]
		r_car = 0.25

		rho_obs, phi_obs = cart2pol(x_obs, y_obs)
		rotation_angle = self.ctrl_ang/100*math.pi/4		
		
		phi_obs -= rotation_angle
		
		x_obs, y_obs = pol2cart(rho_obs, phi_obs)
		
		# Adaptive minimum allowed distance
		#min_dist = self.min_dist + 0.1*abs(self.ctrl_vel)
		min_dist = self.min_dist
		if x_obs < - math.fabs(0.1*y_obs) and math.fabs(y_obs) <  r_car + r_obs + 0.5 and math.sqrt(x_obs**2 + y_obs**2) < r_car + min_dist:
			return True
		else:
			return False
	def publish_danger(self):
		danger_msg = Float32()
		circle_obstacle_found = False
		segment_obstacle_found = False
		for circle in self.circle_obstacles:
			if self.circle_collision(circle):
				circle_obstacle_found = True
				self.obstacle = circle
		for segment in self.segment_obstacles:
			if self.segment_collision(segment):
				segment_obstacle_found = True
		if segment_obstacle_found:
			danger_msg.data = 1

			#des_obstacle_msg = PoseStamped()
			#des_obstacle_msg.pose.position.x = self.obstacle[0]
			#des_obstacle_msg.pose.position.y = self.obstacle[1]
			#des_obstacle_msg.pose.position.z = self.obstacle[2]
			#self.pub_des_obstacle.publish(des_obstacle_msg)

		else:		
			danger_msg.data = 0
		self.pub_danger.publish(danger_msg)

	def publish_danger_from_lidar(self):
		danger_msg = Float32()
		obstacle_found = False
		for i in range(len(self.obstacle_points)):
			x = self.obstacle_points[i][0]
			y = self.obstacle_points[i][1]
			if self.in_danger_zone(x,y):
				obstacle_found = True
		if obstacle_found:
			danger_msg.data = 1
		else:
			danger_msg.data = 0
		self.pub_danger.publish(danger_msg)
		
	def segment_collision(self, segment_obstacle): # A segment_obstacle is a list [(first_point_x, first_point_y), (last_point_x, last_point_y)]		
		first_point_x = segment_obstacle[0][0]
		first_point_y = segment_obstacle[0][1]
		last_point_x = segment_obstacle[1][0]
		last_point_y = segment_obstacle[1][1]
		middle_point_x = (first_point_x + last_point_x)/2
		middle_point_y = (first_point_y + last_point_y)/2
		x_points = [first_point_x, last_point_x, middle_point_x]
		y_points = [first_point_y, last_point_y, middle_point_y]
		'''
		first_rho_obs, first_phi_obs = cart2pol(first_point_x,first_point_y)
		last_rho_obs, last_phi_obs = cart2pol(last_point_x,last_point_y)
		rotation_angle = self.ctrl_ang/100*math.pi/4		
		
		first_phi_obs -= rotation_angle
		last_phi_obs -= rotation_angle
		
		first_point_x, first_point_y = pol2cart(first_rho_obs, first_phi_obs)
		last_point_x, last_point_y = pol2cart(last_rho_obs, last_phi_obs)
	
		middle_point_x = (first_point_x + last_point_x)/2
		middle_point_y = (first_point_y + last_point_y)/2
		'''
		# Adaptive minimum allowed distance
		#min_dist = self.min_dist + 0.1*abs(self.ctrl_vel)
		min_dist = self.min_dist
		r_car = 0.25
		for x, y in zip(x_points, y_points):
			if (x < - math.fabs(0.5*y)) and (math.fabs(y) < (r_car + 0.2)) and ((x**2 + y**2) < (r_car + min_dist)):
				return True		
		return False


	def in_danger_zone(self,x_obs, y_obs):
		r_car = 0.25
		if x_obs < - math.fabs(0.5*y_obs) and math.fabs(y_obs) <  r_car + 0.5 and math.sqrt(x_obs**2 + y_obs**2) < r_car + self.min_dist:
			return True
		else:
			return False
		

	
if __name__ == "__main__":
	rospy.init_node('obstacle_measurement', anonymous=True)
	rate = rospy.Rate(10)
	my_obstacle_measurement = obstacle_measurement()
	while not rospy.is_shutdown():
		#my_obstacle_measurement.parse_obstacles(my_obstacle_measurement.obstacle_msg)
		#my_obstacle_measurement.publish_danger()
		if my_obstacle_measurement.scan_available:
			my_obstacle_measurement.parse_scan(my_obstacle_measurement.scan_msg)
			my_obstacle_measurement.publish_danger_from_lidar()
		rate.sleep()

