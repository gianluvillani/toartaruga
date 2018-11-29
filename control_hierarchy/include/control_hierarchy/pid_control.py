#!/usr/bin/env python

import rospy
import math
from control_algorithm import ControlAlgorithm


class PidControl(ControlAlgorithm):

	def __init__(self):
		self.start_steering = 0
		self.error_i_yaw = 0
		self.error_yaw = 0
		self.old_error_yaw = 0

		self.error_i_distance = 0
		self.error_distance = 0
		self.old_error_distance = 0

	'''
	Inputs:
		car_state : Pose
		target : Pose
		constraints: None

	Outputs:
		lli_control_request
	'''
	def get_control(self, car_state, target, constraints=None):
		pass

	def update_error_i(self):
		self.error_i_yaw += self.error_yaw * self.parameters['Ts']
		self.error_i_distance += self.error_distance * self.parameters['Ts']

	def derivative(self):
		error_d_distance = (self.error_distance - self.old_error_distance)/self.Ts
		error_d_yaw = (self.error_yaw - self.old_error_yaw)/self.parameters['Ts']
		return error_d_yaw, error_d_distance

	def compute_velocity_angular(self, x_car, y_car, yaw_car, x_waypoint, y_waypoint):
		d = math.sqrt((x_car - x_waypoint) ** 2 + (y_car - y_waypoint) ** 2)
		self.old_error_distance = self.error_distance
		self.error_distance = d - self.parameters['target_distance']

		if abs(self.error_distance) < 0.1:
			self.error_distance = 0.0
			self.error_i_distance = 0.0

		yaw_des = math.atan2(y_waypoint - y_car, x_waypoint - x_car)
		self.old_error_yaw = self.error_yaw
		self.error_yaw = yaw_des - yaw_car
		self.update_error_i()

		error_d_yaw, error_d_distance = self.derivative()

		v = self.parameters['K_dis_P'] * self.error_distance + self.parameters['K_dis_D'] * error_d_distance + self.parameters['K_dis_I'] * self.error_I_distance

		delta = self.parameters['K_yaw_P'] * self.error_yaw \
				+ self.parameters['K_yaw_D'] * error_d_yaw +\
				self.parameters['K_yaw_I'] * self.error_i_yaw

		if delta > math.pi / 4:
			delta = math.pi / 4

		if delta < -math.pi / 4:
			delta = -math.pi / 4

		if self.error_yaw > math.pi:
			delta = -self.parameters['K_yaw_P'] * (2 * math.pi - self.error_yaw) +\
					self.parameters['K_yaw_D'] * error_d_yaw +\
					self.parameters['K_yaw_I'] * self.error_i_yaw
			if delta > math.pi / 4:
				delta = math.pi / 4
			if delta < -math.pi / 4:
				delta = -math.pi / 4

		if self.error_yaw < -math.pi:
			delta = self.parameters['K_yaw_P'] * (2 * math.pi + self.error_yaw) +\
					self.parameters['K_yaw_D'] * error_d_yaw +\
					self.parameters['K_yaw_I'] * self.error_i_yaw

			if delta > math.pi / 4:
				delta = math.pi / 4

			if delta < -math.pi / 4:
				delta = -math.pi / 4

		return delta, v
