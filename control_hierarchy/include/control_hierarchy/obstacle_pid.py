#!/usr/bin/env python

import math
import rospy
import numpy as np
from control_algorithm import ControlAlgorithm
from pure_pursuit import PurePursuit
from spline_interpolation import Spline2D
from low_level_interface.msg import lli_ctrl_request
from geometry_msgs.msg import Point

class CircularAvoidance(ControlAlgorithm):

	def __init__(self):
		pass

	'''
	Inputs:
		car_state - PoseStamped
		target - Path : reference path
		Parameters:
			circle_obstacles: List of Obstacles of the Circle variation IN THE LOCAL FRAME
			line_obstacles: List of Obstacles of the Line variation IN THE LOCAL FRAME
			turning_radius: desired turning radius
			radius_padding: [m] safety distance to an obstacle
	Outputs:
		lli_ctrl_request - the desired control signal
	'''
	def get_control(self, car_state, target, parameters={'circle_obstacles':[], 'line_obstacles':[], 'turning_radius':0.4, 'radius_padding':0.2}):
		if car_state == None or target == None:
			msg = lli_ctrl_request()
			msg.steering = 0
			msg.velocity = 0
			return msg
		self.parameters = parameters
		self.target = target
		self.car_state = self.pose_to_xy_yaw(car_state)
		self.car_point = Point()
		self.car_point.x = self.car_state['x']
		self.car_point.y = self.car_state['y']

		feasible_path = False
		current_path = self.target
		while not feasible_path:
			obstacle = self.get_closest_obstacle_on_path(current_path)
			if obstacle == None:
				feasible_path = True
			else:
				arc = self.get_arc(obstacle)
				if arc == None:
					feasible_path = True
				else:
					current_path = arc

		PP_control = PurePursuit()
		control = PP_control.get_control(car_state, current_path)
		if abs(control.steering) == 100:
			control.velocity = 0
			control.steering = 0
		return control


	def get_arc(self, circle):
		pos_arc = self.turning_radius(circle, 1)
		neg_arc = self.turning_radius(circle, -1)
		target_radius = self.parameters['turning_radius']

		if pos_arc < target_radius and -neg_arc < target_radius:
			if pos_arc > -neg_arc:
				start_point = circle
				start_point.x += pos_arc
				return self.arc_from_points(self.car_point, start_point, circle.center)
			else:
				start_point = circle
				start_point.x += neg_arc
				return self.arc_from_points(self.car_point, start_point, circle.center)
		else:
			return None

	def arc_from_points(self, point_a, center, point_b, npoints=100):
		angle = math.atan2(point_b.y - center.y, point_b.x - center.x)
		d_angle = angle/npoints
		path = []
		for i in range(npoints):
			point = point_a
			point.y += math.sin(i*d_angle)*(point_a.x - center.x)
			point.x += math.cos(i*d_angle)*(point_a.x - center.x)
			path.append(point)
		return path


	def get_closest_obstacle_on_path(self, path):
		circles = self.transform_obstacles_frame(self.parameters['circle_obstacles'], self.car_state)
		circles_on_path = []
		for c, local in zip(circles, self.parameters['circle_obstacles']):
			center = c.center #Point
			radius = c.true_radius + self.parameters['radius_padding']

			for point in path.poses:
				if self.point_msg_distance(point, center) < radius & local.center.x > 0:
					circles_on_path.append(local)
		min_distance = 1000
		min_c = None

		for c in circles_on_path:
			if self.point_msg_distance(c.center, self.car_state) < min_distance:
				min_distance = self.point_msg_distance(c.center, self.car_point)
				min_c = c
		return min_c


	def turning_radius(self, circle, arc_direction):
		min_radius = 0.2 * arc_direction
		current_radius = self.point_msg_distance(self.car_point, circle.center) * arc_direction
		max_radius = None

		current_distance = self.distance_between_circles(self.car_point, current_radius, circle)
		threshold = 0.01

		while abs(current_distance) > threshold:
			current_distance = self.distance_between_circles(self.car_point, current_radius, circle)
			if current_distance < 0:
				max_radius = current_radius
				current_radius = min_radius + (current_radius - min_radius)/2

			if current_distance > 0:
				if max_radius == None:
					min_radius = current_radius
					current_radius = current_radius*2
				else:
					min_radius = current_radius
					current_radius = current_radius + (max_radius-current_radius)/2
		return current_radius


	def distance_between_circles(self, point, radius, circle):
		new_center = point
		new_center.y += radius
		dist = self.point_msg_distance(circle.center, new_center)
		dist -= radius
		dist -= circle.radius

		return dist
