#!/usr/bin/env python

import rospy
from control_algorithm import ControlAlgorithm

class PidControl(ControlAlgorithm):

	def __init__(self):
		pass

	def get_control(self, car_state, target, constraints=None):
		pass