#!/usr/bin/env

from control_algorithm import ControlAlgorithm
from low_level_interface.msg import lli_ctrl_request

class StopAlgorithm(ControlAlgorithm):

	def __init__(self):
		self.name = "stop"
		pass


	def get_control(self, car_state, target, parameters=None):
		control = lli_ctrl_request()
		control.velocity = -10
		control.steering = 0

		return control
