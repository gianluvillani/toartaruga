#!/usr/bin/env python

import math
import rospy
import numpy as np
from control_algorithm import ControlAlgorithm
from spline_interpolation import Spline2D
from low_level_interface.msg import lli_ctrl_request

class ObstaclePid(ControlAlgorithm):

	def __init__(self):
		pass

	'''
	Inputs:
		car_state - PoseStamped
		target - Path : reference path
		Parameters:
			circle_obstacles: List of Obstacles of the Circle variation IN THE LOCAL FRAME
			line_obstacles: List of Obstacles of the Line variation IN THE LOCAL FRAME
			theta: desired turning angle
			radius_padding: [m] safety distance to an obstacle
	Outputs:
		lli_ctrl_request - the desired control signal
	'''
	def get_control(self, car_state, target, parameters={'circle_obstacles':[], 'line_obstacles':[], 'theta':math.pi/8, 'radius_padding':0.2}):
		self.parameters = parameters
		self.target = target
		self.car_state = self.pose_to_xy_yaw(car_state)

	def get_closest_obstacle_on_path(self):
		circles = self.transform_obstacles_frame(self.parameters['circle_obstacles'], self.car_state)
		circles_on_path = []
		for c, local in zip(circles, self.parameters['circle_obstacles']):
			center = c.center #Point
			radius = c.true_radius + self.parameters['radius_padding']

			for point in self.target:
				if self.point_msg_distance(point, center) < radius & local.center.x > 0:
					circles_on_path.append(local)

		for c in circles_on_path:
			pass