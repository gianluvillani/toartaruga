#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle

from low_level_interface.msg import lli_ctrl_request
from pure_pursuit import PurePursuit
from pid_control import PidControl
from stop_algorithm import StopAlgorithm
from obstacle_pid import CircularAvoidance

class Dispatcher:

	def __init__(self, rate=30):
		self.available_controls = {'pid':self.init_pid, 'pure_pursuit':self.init_pp, 'stop':self.init_stop, 'arc_avoidance':self.init_arc}
		self.current_control_key = rospy.get_param(rospy.get_name() + '/selected_controller')
		self.rate = rospy.Rate(rate)

		self.algorithm = None
		self.car_state = None
		self.control_target = None
		self.circle_obstacles = []
		self.segment_obstacles = []
		self.parameters = {'v':0, 'steering':0, 'k_lookahead':1, 'l':2, 'circle_obstacles':[], 'line_obstacles':[], 'turning_radius':0.4, 'radius_padding':0.2}

		self.switch_control(self.current_control_key)

		#Subscribers
		self.sub_car_state = rospy.Subscriber(rospy.get_param('/dispatcher/car_pose_topic'), PoseStamped, self.car_state_callback)
		self.sub_control_target = None
		self.sub_obstacles = None

		#Publishers
		self.pub_control_signal = rospy.Publisher(rospy.get_param('/dispatcher/control_signal_topic'), lli_ctrl_request)

	'''
		Runs the new initialization for the new algorithm.
	'''
	def switch_control(self, new_control_key):
		self.available_controls[new_control_key]()


	def publish_control(self):
		if self.car_state == None or self.control_target == None:
			control_signal = lli_ctrl_request()
			control_signal.steering = 0
			control_signal.velocity = 0
			self.pub_control_signal.publish(control_signal)
			return
		control_signal = self.algorithm.get_control(car_state=self.car_state, target=self.control_target, parameters=self.parameters)
		# Pure pursuit parameters
		self.parameters['v'] = control_signal.velocity
		self.parameters['steering'] = control_signal.steering

		# Avoidance parameters
		self.parameters['circle_obstacles'] = self.circle_obstacles
		self.parameters['line_obstacles'] = self.segment_obstacles
		#PID parameters

		#And publish
		self.pub_control_signal.publish(control_signal)


	'''
		Initializes pid.
	'''
	def init_pid(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/waypoint_topic'), PoseStamped, self.control_target_callback)
		self.algorithm = PidControl()

	'''
		Initializes pure pursuit.
	'''
	def init_pp(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/path_topic'), Path, self.control_target_callback)
		self.algorithm = PurePursuit()

	'''
		Initializes stopping sequence.
	'''
	def init_stop(self):
		self.sub_control_target = None
		self.algorithm = StopAlgorithm()

	def init_arc(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/path_topic'), Path, self.control_target_callback)
		self.sub_obstacles = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/obstacles_topic'), Obstacles, self.obstacles_callback)
		self.algorithm = CircularAvoidance()


	def car_state_callback(self, msg):
		self.car_state = msg

	def control_target_callback(self, msg):
		self.control_target = msg

	def obstacles_callback(self, msg):
		self.circle_obstacles = msg.circles
		self.segment_obstacles = msg.segments

if __name__=='__main__':
	rospy.init_node('dispatcher', anonymous=True)
	rospy.spin()
