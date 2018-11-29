#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from low_level_interface.msg import lli_ctrl_request
from pure_pursuit import PurePursuit
from pid_control import PidControl
from stop_algorithm import StopAlgorithm

class Dispatcher:

	def __init__(self, rate=30):
		self.available_controls = {'pid':self.init_pid, 'pure_pursuit':self.init_pp, 'stop':self.init_stop}
		self.current_control_key = rospy.get_param(rospy.get_name() + 'chosen_control')
		self.rate = rospy.Rate(rate)

		self.algorithm = None
		self.car_state = None
		self.control_target = None
		self.parameters = {'v':0, 'steering':0, 'k_lookahead':1, 'l':2}

		self.switch_control(self.current_control_key)

		#Subscribers
		self.sub_car_state = rospy.Subscriber(rospy.get_param(rospy.get_name() + 'car_pose_topic'), self.car_state_callback)
		self.sub_control_target = None

		#Publishers
		self.pub_control_signal = rospy.Publisher(rospy.get_param(rospy.get_name() + 'control_signal_topic'), lli_ctrl_request)

	'''
		Runs the new initialization for the new algorithm.
	'''
	def switch_control(self, new_control_key):
		self.available_controls[new_control_key]()


	def publish_control(self):
		control_signal = self.algorithm.get_control(car_state=self.car_state, target=self.control_target, parameters=self.parameters)
		# Pure pursuit parameters
		self.parameters['v'] = control_signal.velocity
		self.parameters['steering'] = control_signal.steering

		#PID parameters

		#And publish
		self.pub_control_signal.publish(control_signal)


	'''
		Initializes pid.
	'''
	def init_pid(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + 'waypoint_topic'), self.control_target_callback)
		self.algorithm = PidControl()

	'''
		Initializes pure pursuit.
	'''
	def init_pp(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + 'path_topic'), self.control_target_callback)
		self.algorithm = PurePursuit()

	'''
		Initializes stopping sequence.
	'''
	def init_stop(self):
		self.sub_control_target = None
		self.algorithm = StopAlgorithm()


	def car_state_callback(self, msg):
		self.car_state = msg

	def control_target_callback(self, msg):
		self.control_target = msg