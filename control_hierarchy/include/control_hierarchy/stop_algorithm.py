#!/usr/bin/env
import rospy
from control_algorithm import ControlAlgorithm
from low_level_interface.msg import lli_ctrl_request

class StopAlgorithm(ControlAlgorithm):

	def __init__(self):
		self.name = "stop"
		pass


	def get_control(self, car_state, target, parameters={'steering':0}):
		control = lli_ctrl_request()
		control.velocity = -100
		control.steering = parameters['steering']
		rospy.logerr("Stop algorithm")

		return control
