#! /usr/bin/env python

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import os
import time
import random
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from low_level_interface.msg import lli_ctrl_request
from tf.transformations import euler_from_quaternion
from controller import controller


class PID_distance(controller):
	"""
	PID_distance controller

	Program flow:

	while (True):
		(1) Subscribe to the cars' positions and get the distance error and store it in self.error
		(2) Update the integral of the error in self.integral_error
		(3) Calculate the error derivative with self.error and self.old_error
		(4) Compute the control signal with the PID-parameters
		(5) Publish the control signal
	"""

	def __init__(self, P, I=0, D=0, Ts=0.1, goal_distance=0.5):
		"""
		p - Proportional part
		i - Integral part
		d - Derivative part
		Ts - Sampling time
		goal_distance - How far away we want to be to the car in front.
		integral_error - Integral of error
		old_error - The error at the previous time step
		error - Current error
		"""
		self.P = P
		self.I = I
		self.D = D
		self.Ts = Ts
		self.goal_distance = goal_distance
		self.integral_error = 0
		self.old_error = 0
		self.error = 0

		# Publishers/Subscribers NOT THE CORRECT ONES
		self.pub_speed_control = rospy.Publisher('/lli/ctrl_request', lli_ctrl_request)
		self.sub_pose = rospy.Subscriber('/simulator/odom', Odometry, self.save_state)
		self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.save_path)

	def get_error(self, msg):
		"""
		Subscriber callback function

		TODO: Calculate the distance between cars from the msg
		and compare this to .
		Store error in the instance error

		self.old_error = self.error
		self.error = new_error
		"""
		pass

	def get_control_signal(self):
		"""
		Computes control signal with the PID-parameters
		"""
		derivative = self.derivative()
		control_signal = self.P * self.error + self.I * self.integral_error + self.D * derivative
		return control_signal

	def derivative(self):
		"""
		Discrete derivative of error
		"""
		derivative = (self.error - self.old_error) / self.Ts
		return derivative

	def update_error_integral(self):
		"""
		Updates discrete integral of error

		TODO: Anti windup
		"""
		self.integral_error += self.error * self.Ts
