#!/usr/bin/env python

import tf
import tf2_ros
import rospy
import math
import numpy as np
import geometry_msgs.msg
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from low_level_interface.msg import lli_ctrl_request
#TODO: Use low_level_interface on NVIDIA
from geometry_msgs.msg import Twist
from spline_interpolation import Spline2D
from obstacle_detector.msg import Obstacles

class Replanner:
	def __init__(self, safety_distance = 0.1):
		"""
		(x_car, y_car) = Position of the car
		(cx, cy) = The current path
		(x_obstacle, y_obstacle) = Position of the obstacle (global coordinates)
		r = Radius of the obstacle
		safety_distance = Distance margin to the obstacle
		"""
		#self.x_car = x_car
		#self.y_car = y_car
		self.x_obstacle_global = 0
		self.y_obstacle_global = 0
		self.r = 1
		self.safety_distance = safety_distance
		self.get_course()
		self.obstacle_passed = True
		self.path_available = False
		self.state_available = False
		self.danger = 0

		# Access rosparams
		self.obstacles_top = rospy.get_param(rospy.get_name() + '/obstacles_topic')
		self.des_obstacle_top = rospy.get_param(rospy.get_name() + '/des_obstacle_topic')
		self.danger_top = rospy.get_param(rospy.get_name() + '/danger_topic')
		self.car_pose_top = rospy.get_param(rospy.get_name() + "/car_pose_topic")
		self.path_top = rospy.get_param(rospy.get_name() + "/path_topic")
		self.replanner_path_top = rospy.get_param(rospy.get_name() + "/replanner_path_topic")
		self.leader_path_top = rospy.get_param(rospy.get_name() + "/leader_path_topic")
		
		# Publishers/Subscriber
		self.sub_des_obstacle = rospy.Subscriber(self.des_obstacle_top, PoseStamped, self.save_des_obstacle)
		self.sub_obstacles = rospy.Subscriber(self.obstacles_top, Obstacles, self.save_obstacles)
		self.sub_danger = rospy.Subscriber(self.danger_top, Float32, self.save_danger)
		self.sub_car_pose = rospy.Subscriber(self.car_pose_top, PoseStamped, self.save_state)
		self.sub_path = rospy.Subscriber(self.path_top, Path, self.save_path)
		#self.sub_leader_path = rospy.Subscriber(self.leader_path_top, Path, self.save_leader_path)
		self.pub_path = rospy.Publisher(self.replanner_path_top, Path)
		self.pub_marker = rospy.Publisher('/marker', PoseStamped)

	def save_path(self, path_msg):
		self.path = path_msg
		self.path_available = True

	def save_danger(self, danger_msg):
		self.danger = danger_msg.data

	def save_state(self, state_msg):
		self.state = state_msg
		self.state_available = True

	def save_obstacles(self, obstacle_msg):
		self.obstacle_msg = obstacle_msg

	def save_des_obstacle(self, des_obstacle_msg):
		
		self.x_obstacle = des_obstacle_msg.pose.position.x
		self.y_obstacle = des_obstacle_msg.pose.position.y
		self.r = des_obstacle_msg.pose.position.z
		self.transform_des_obstacle_frame()
		

	def parse_obstacles(self, obstacle_msg):
		self.segment_obstacles = []	
		self.circle_obstacles = []	
		for segment in obstacle_msg.segments:
			self.segment_obstacles.append([(segment.first_point.x, segment.first_point.y), (segment.last_point.x, segment.last_point.y)])
		for circle in obstacle_msg.circles:
			self.circle_obstacles.append((circle.center.x , circle.center.y, circle.radius))

	def transform_des_obstacle_frame(self):
			if self.state_available:
				alpha = math.atan2(self.y_obstacle, self.x_obstacle)
				theta = math.pi + self.yaw_car
				rho = math.sqrt(self.x_obstacle**2 + self.y_obstacle**2)
				self.x_obstacle_global = self.x_car + rho*math.cos(alpha + theta)
				self.y_obstacle_global = self.y_car + rho*math.sin(alpha + theta)				
			
				marker_msg = PoseStamped()
				marker_msg.header.frame_id = 'qualisys'
				marker_msg.pose.position.x = self.x_obstacle_global
				marker_msg.pose.position.y = self.y_obstacle_global
	
				self.pub_marker.publish(marker_msg)
		
	
	def transform_obstacles_frame(self):
		self.circle_obstacles_global = []
		if self.state_available:
			for obstacle in self.circle_obstacles:
				x_obstacle = obstacle[0]
				y_obstacle = obstacle[1]			
				alpha = math.atan2(self.y_obstacle, self.x_obstacle)
				theta = math.pi + self.yaw_car
				rho = math.sqrt(self.x_obstacle**2 + self.y_obstacle**2)
				x_obstacle_global = self.x_car + rho*math.cos(alpha + theta)
				y_obstacle_global = self.y_car + rho*math.sin(alpha + theta)
				self.circle_obstaclels_global.append((x_obstacle_global,y_obstacle_global,obstacle[2]))
	

	def parse_state(self, state_msg):
		self.x_car = state_msg.pose.position.x
	        self.y_car = state_msg.pose.position.y
		orientation_q = state_msg.pose.orientation
	        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	        self.yaw_car = euler_from_quaternion(orientation_list)[2]
	
	def check_obstacle_passed(self):
		if ((self.x_car - self.x_obstacle_global)**2 + (self.y_car - self.y_obstacle_global)**2) > self.r + 0.5:
			self.obstacle_passed = True
			
	def check_obstacle_passed1(self):
		dx_car = [self.x_car - icx for icx in self.cx]
		dy_car = [self.y_car - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx_car, dy_car)]
		
	def new_path1(self):
		"""
		Returns True if a new path should be created.
		"""
		close_obstacle = False
		obstacle_on_path = False
		for x, y in zip(self.cx, self.cy):
			if math.sqrt((x - self.x_obstacle_global) ** 2 + (y - self.y_obstacle_global) ** 2) < 0.3 + self.r: #self.r + self.safety_distance:
				obstacle_on_path = True
				break
		return obstacle_on_path
		
		#distance_to_car = math.sqrt(self.x_obstacle**2 + self.y_obstacle**2) + self.r + self.safety_distance
		#if distance_to_car < 0.2 + self.r + self.safety_distance:
		#	close_obstacle = True
		#if close_obstacle and obstacle_on_path:
		#	return True
		#else:
		#	return False

	def new_path(self):
		"""
		Returns True if a new path should be created.
		"""
		close_obstacle = False
		obstacle_on_path = False
		for x, y in zip(self.cx, self.cy):
			if math.sqrt((x - self.x_obstacle_global) ** 2 + (y - self.y_obstacle_global) ** 2) < 0.3 + self.r: #self.r + self.safety_distance:
				obstacle_on_path = True
				break
		return obstacle_on_path
		
		#distance_to_car = math.sqrt(self.x_obstacle**2 + self.y_obstacle**2) + self.r + self.safety_distance
		#if distance_to_car < 0.2 + self.r + self.safety_distance:
		#	close_obstacle = True
		#if close_obstacle and obstacle_on_path:
		#	return True
		#else:
		#	return False	

	def get_course(self):
		
		t = np.arange(0, math.pi*2, 0.01)
		x = np.cos(t)
		y = np.sin(t)
		
		"""
		x = np.arange(0, 20, 0.01)
		y = np.zeros([2000])
		"""
		#plt.plot(x,y)
		#plt.show()
		self.cx = list(x)
		self.cy = list(y)

	def find_indices(self):
		"""
		Obtains indices of where the old map should be replaced.
		"""
		distance = math.sqrt((self.x_car - self.x_obstacle_global)**2 + (self.y_car - self.y_obstacle_global)**2)
		dx_car = [self.x_car - icx for icx in self.cx]
		dy_car = [self.y_car - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx_car, dy_car)]
		start_index = d.index(min(d))
		dx_obstacle = [self.x_obstacle_global - icx for icx in self.cx]
		dy_obstacle = [self.y_obstacle_global - icy for icy in self.cy]
	    	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx_obstacle, dy_obstacle)]
		obstacle_index = d.index(min(d))
		dist = min(d)
		stop_index = (2*obstacle_index - start_index) % len(self.cx)
		self.obs = obstacle_index

		rospy.logerr("start_index=%s, stop_index=%s, cx=%s, cy=%s, obstacle_index=%s", start_index, stop_index, self.cx[obstacle_index], self.cy[obstacle_index], obstacle_index)
		return start_index, obstacle_index, stop_index, dist
	
	def get_waypoint_direction(self, obs_ind):
		"""
		Returns unit vector of the direction the new waypoint should be placed
		This vector is orthonormal to the path where the obstacle is.
		It is directed towards the shortest path around the obstacle.
		"""
		path_direction = np.array([self.cx[(obs_ind + 1) % len(self.cx)] - self.cx[obs_ind], self.cy[(obs_ind + 1) % len(self.cy)] - self.cy[obs_ind]])
		waypoint_direction = np.zeros(2)
		waypoint_direction[0] = -path_direction[1]
		waypoint_direction[1] = path_direction[0]
		waypoint_direction = waypoint_direction/np.linalg.norm(waypoint_direction, ord = 2)
		
		delta_x = self.cx[(obs_ind + 1) % len(self.cx)] - self.cx[obs_ind]
		delta_y = self.cy[(obs_ind + 1) % len(self.cy)] - self.cy[obs_ind]
		
		if delta_x != 0:
			m = delta_y/delta_x
			q = self.cy[obs_ind] - m*self.cx[obs_ind]
			if self.y_obstacle_global > m*self.x_obstacle_global + q:
				direction = 'clockwise'
			else:
				direction = 'counter_clockwise' 
		else:
			if self.x_obstacle_global <= self.cx[obs_ind]:
				direction = 'clockwise'
			else:
				direction = 'counter_clockwise'

		if direction == 'clockwise':
			waypoint_direction *= -1

		return waypoint_direction


		
		
	def get_new_path(self):
		"""
		Replaces old path with new path avoiding the obstacle
		""" 
		self.new_cx = []
		self.new_cy = []
		start_index, obstacle_index, stop_index, dist = self.find_indices()
		waypoint_direction = self.get_waypoint_direction(obstacle_index)
		new_waypoint = np.array([self.x_obstacle_global, self.y_obstacle_global]) + waypoint_direction*(self.r + self.safety_distance)
		x_points = [self.cx[start_index], new_waypoint[0], self.cx[stop_index]]
		y_points = [self.cy[start_index], new_waypoint[1], self.cy[stop_index]]
		cx, cy = self.spline_waypoints(x_points, y_points)
		if stop_index < start_index:
			self.new_cx = self.cx[:start_index] + cx + self.cx[stop_index:start_index]
			self.new_cy = self.cy[:start_index] + cy + self.cy[stop_index:start_index]
		else:
			self.new_cx = self.cx[:start_index] + cx + self.cx[stop_index:]
			self.new_cy = self.cy[:start_index] + cy + self.cy[stop_index:]
		

		
	
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
			pose.header.frame_id = 'qualisys'
			pose.pose.position.x = x
			pose.pose.position.y = y
			pose.pose.position.z = z_init
			z_init+=0.001	
			path_msg.poses.append(pose)
		path_msg.header.frame_id = 'qualisys'
		return path_msg


"""
a = Replanner()

if a.new_path():
	print('ja')
	a.get_new_path()


plt.plot(a.cx, a.cy)
circle1=plt.Circle((a.x_obstacle,a.y_obstacle),a.r,color='r')
plt.gcf().gca().add_artist(circle1)
circle2=plt.Circle((a.x_obstacle,a.y_obstacle),a.r+a.safety_distance, color = 'k', fill = False)
plt.gcf().gca().add_artist(circle2)	
plt.plot(a.cx, a.cy)
#plt.plot(new_waypoint[0], new_waypoint[1], 'xg')
plt.show()
"""

if __name__ == '__main__':
	rospy.init_node('path_replanner')
	a = Replanner(safety_distance = 0.1)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		if a.path_available and a.state_available:
			a.parse_state(a.state)
			a.parse_path(a.path)
			if a.danger > 0.5 and a.obstacle_passed and a.new_path():
				a.get_new_path()
				a.obstacle_passed = False
				rospy.logerr("in if in planner")
				path_msg = a.coordinates_to_msg(a.new_cx, a.new_cy)
			elif not a.obstacle_passed:
				rospy.logerr("in elif in planner")
				a.check_obstacle_passed()
				path_msg = a.coordinates_to_msg(a.new_cx, a.new_cy)
			else:
				rospy.logerr("in else in planner")
				path_msg = a.coordinates_to_msg(a.cx, a.cy)
			a.pub_path.publish(path_msg)
			rate.sleep()




	
	
		


