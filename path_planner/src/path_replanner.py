#!/usr/bin/env python

import tf
import tf2_ros
import rospy
import math
import numpy as np
import geometry_msgs.msg
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from low_level_interface.msg import lli_ctrl_request
#TODO: Use low_level_interface on NVIDIA
from geometry_msgs.msg import Twist
from car_model import KinematicBicycle
from spline_interpolation import Spline2D

class Replanner:
	def __init__(self, x_car = 0, y_car= 0, x_obstacle = 3, y_obstacle = 9, r = 2, safety_distance = 0.5):
		"""
		(x_car, y_car) = Position of the car
		(cx, cy) = The current path
		(x_obstacle, y_obstacle) = Position of the obstacle (global coordinates)
		r = Radius of the obstacle
		safety_distance = Distance margin to the obstacle
		"""
		self.x_car = x_car
		self.y_car = y_car
		self.x_obstacle = x_obstacle
		self.y_obstacle = y_obstacle
		self.r = r
		self.safety_distance = safety_distance
		self.get_course()
		
		"""
		Subscribers:
			- Pose of car
			- Current path
		Publisher:
			- Altered path
		"""
		#self.sub_pose = rospy.Subscriber('/simulator/odom', Odometry, self.save_state)
		#self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.save_path)
		#self.pub_path = rospy.Publisher('/SVEA2/path', Path)

	def get_course(self):
		x = np.arange(20)
		y = 3*np.arange(20)
		self.cx = list(x)
		self.cy = list(y)

	def find_indices(self):
		"""
		Obtains indices of where the old map should be replaced.
		"""
		distance = math.sqrt((self.x_car - self.x_obstacle)**2 + (self.y_car - self.y_obstacle)**2)
		dx_car = [self.x_car - icx for icx in self.cx]
		dy_car = [self.y_car - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx_car, dy_car)]
		start_index = d.index(min(d))
		dx_obstacle = [self.x_obstacle - icx for icx in self.cx]
		dy_obstacle = [self.y_obstacle - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx_obstacle, dy_obstacle)]
		obstacle_index = d.index(min(d))
		dist = min(d)
		stop_index = 2*obstacle_index - start_index
		return start_index, obstacle_index, stop_index, dist
	
	def get_waypoint_direction(self, obs_ind):
		"""
		Returns unit vector of the direction the new waypoint should be placed
		This vector is orthonormal to the path where the obstacle is.
		It is directed towards the shortest path around the obstacle.
		"""
		path_direction = np.array([self.cx[obs_ind + 1] - self.cx[obs_ind], self.cy[obs_ind+1] - self.cy[obs_ind]])
		waypoint_direction = np.zeros(2)
		waypoint_direction[0] = -path_direction[1]
		waypoint_direction[1] = path_direction[0]
		waypoint_direction = waypoint_direction/np.linalg.norm(waypoint_direction, ord = 2)
		
		delta_x = self.cx[obs_ind + 1] - self.cx[obs_ind]
		delta_y = self.cy[obs_ind + 1] - self.cy[obs_ind]
		
		if delta_x != 0:
			m = delta_y/delta_x
			q = self.cy[obs_ind] - m*self.cx[obs_ind]
			print(m, q)
			print(self.y_obstacle, m*self.x_obstacle + q)
			if self.y_obstacle > m*self.x_obstacle + q:
				direction = 'clockwise'
			else:
				direction = 'counter_clockwise' 
		else:
			if self.x_obstacle <= self.cx[obs_ind]:
				direction = 'clockwise'
			else:
				direction = 'counter_clockwise'
		print(direction)
		if direction == 'clockwise':
			return -waypoint_direction
		else:
			return waypoint_direction


		
		
	def new_path(self):
		"""
		Replaces old path with new path avoiding the obstacle
		""" 
		start_index, obstacle_index, stop_index, dist = self.find_indices()
		waypoint_direction = self.get_waypoint_direction(obstacle_index)
		new_waypoint = np.array([self.x_obstacle, self.y_obstacle]) + waypoint_direction*(self.r + self.safety_distance)
		x_points = [self.cx[start_index], new_waypoint[0], self.cx[stop_index]]
		y_points = [self.cy[start_index], new_waypoint[1], self.cy[stop_index]]
		cx, cy = self.spline_waypoints(x_points, y_points)
		plt.plot(self.cx, self.cy)
		self.cx = self.cx[:start_index] + cx + self.cx[stop_index:]
		self.cy = self.cy[:start_index] + cy + self.cy[stop_index:]
		circle1=plt.Circle((self.x_obstacle,self.y_obstacle),self.r,color='r')
		plt.gcf().gca().add_artist(circle1)
		circle2=plt.Circle((self.x_obstacle,self.y_obstacle),self.r+self.safety_distance, color = 'k', fill = False)
		plt.gcf().gca().add_artist(circle2)	
		plt.plot(self.cx, self.cy)
		plt.plot(new_waypoint[0], new_waypoint[1], 'xg')
		plt.show()

	
	def spline_waypoints(self, x, y):
		"""
		Generates a path between the newly created waypoints
		"""
		a = sp = Spline2D(x, y)
		s = np.arange(0, sp.s[-1], 0.1)
		rx, ry, ryaw, rk = [], [], [], []
		for i_s in s:
			ix, iy = sp.calc_position(i_s)
			rx.append(ix)
			ry.append(iy)
			ryaw.append(sp.calc_yaw(i_s))
			rk.append(sp.calc_curvature(i_s))
		return rx,ry

	def parse_path(self, path_msg):
		# TODO: This fix is just to debug pure_pursuit, find a solution
		#print "PATH PUBLISHED"				
		self.cx = []
		self.cy = []
		for pose in path_msg.poses:
			self.cx.append(pose.pose.position.x)
			self.cy.append(pose.pose.position.y)

	def coordinates_to_msg(self, cx, cy):
		path_msg = Path()
		z_init = 0
		for x, y in zip(cx, cy):
			pose = PoseStamped()
			pose.header.frame_id = 'odom'
			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = z_init
			z_init+=0.001	
			path_msg.poses.append(pose)
		path_msg.header.frame_id = 'odom'
		return path_msg


if __name__ == '__main__':
	a = Replanner()
	a.new_path()




	
	
		


