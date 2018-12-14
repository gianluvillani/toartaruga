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
import math

class Dispatcher:

	def __init__(self, rate=30):
		self.available_controls = {'pid':self.init_pid, 'pure_pursuit':self.init_pp, 'stop':self.init_stop, 'arc_avoidance':self.init_arc}
		self.current_control_key = rospy.get_param(rospy.get_name() + '/selected_controller')
		self.sub_leader_pose = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/waypoint_topic'), PoseStamped, self.save_leader_pose)
		self.rate = rospy.Rate(rate)
		self.leader_pose_available = False

		self.algorithm = None
		self.car_state = None
		self.control_target = {}
		self.circle_obstacles = []
		self.segment_obstacles = []
		self.parameters = {'v':0, 'steering':0, 'k_lookahead':0.4 , 'l':0.2, 'circle_obstacles':[], 'line_obstacles':[], 'turning_radius':0.4, 'radius_padding':0.2, 'target_distance':0.65, 'Ts':0.0125, 'K_yaw_P':1, 'K_yaw_D':0, 'K_yaw_I':0, 'd_des':1, 'K_dis_P':32, 'K_dis_D':1.9, 'K_dis_I':0.0}

		self.switch_control(self.current_control_key)

		#Subscribers
		self.sub_car_state = rospy.Subscriber(rospy.get_param('/dispatcher/car_pose_topic'), PoseStamped, self.car_state_callback)
		self.sub_control_target = None
		self.sub_obstacles = None

		#fddffers
		self.pub_control_signal = rospy.Publisher(rospy.get_param('/dispatcher/control_signal_topic'), lli_ctrl_request)

	'''
		Runs the new initialization for the new algorithm.
	'''
	def switch_control(self, new_control_key):
		self.available_controls[new_control_key]()

	def save_leader_pose(self, pose_msg):
		self.leader_pose = pose_msg
		self.leader_pose_available = True
 

	def publish_control(self):
		if self.car_state == None or self.control_target == None:
			control_signal = lli_ctrl_request()
			control_signal.steering = 0
			control_signal.velocity = 0
			#rospy.logerr("Dispatcher: missing path or car state")
			self.pub_control_signal.publish(control_signal)
			return
		if self.leader_pose_available and self.algorithm.name == 'pure_pursuit':
			if math.hypot(self.leader_pose.pose.position.x - self.car_state.pose.position.x,
				      self.leader_pose.pose.position.y - self.car_state.pose.position.y) < 0.7:
				
				control_signal = lli_ctrl_request()
				control_signal.steering = 0
				control_signal.velocity = 0
				#rospy.logerr("Dispatcher: missing path or car state")
				self.pub_control_signal.publish(control_signal)
				return
		#rospy.logerr("Dispatcher: publishing the control signals")
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
		try:
			self.sub_control_target.unregister()
		except:
			pass
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/waypoint_topic'), PoseStamped, self.control_target_callback_pid)
		self.algorithm = PidControl()
#		rospy.logerr("COntroller dispatch, init: " + str(self.algorithm.name))

		'''
		Initializes pure pursuit.
		'''
	def init_pp(self):
		try:
			self.sub_control_target.unregister()
		except:
			pass
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/path_topic'), Path, self.control_target_callback_pp)
		self.algorithm = PurePursuit()
		rospy.logerr("Controller dispatch, init: " + str(self.algorithm.name))
		rospy.logerr("Control target: " + str(self.sub_control_target))
		'''
		Initializes stopping sequence.
		'''
	def init_stop(self):
		self.algorithm = StopAlgorithm()

	def init_arc(self):
		self.sub_control_target = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/path_topic'), Path, self.control_target_callback)
		self.sub_obstacles = rospy.Subscriber(rospy.get_param(rospy.get_name() + '/obstacles_topic'), Obstacles, self.obstacles_callback)
		self.algorithm = CircularAvoidance()


	def car_state_callback(self, msg):
		self.car_state = msg

	def control_target_callback_pid(self, msg):
		self.control_target['pid'] = msg
		#rospy.logerr("In dispatcher: control target = %s", self.control_target)

	def control_target_callback_pp(self, msg):
		self.control_target['pure_pursuit'] = msg

	def obstacles_callback(self, msg):
		self.circle_obstacles = msg.circles
		self.segment_obstacles = msg.segments

if __name__=='__main__':
	rospy.init_node('dispatcher', anonymous=True)
	rospy.spin()
