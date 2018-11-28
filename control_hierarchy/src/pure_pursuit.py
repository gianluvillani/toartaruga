#!/usr/bin/env python

import rospy
from control_algorithm import ControlAlgorithm
from tf.transformations import euler_from_quaternion


class PurePursuit(ControlAlgorithm):

	def __init__(self):
		pass

	'''
	Inputs:
		car_state : Odometry
		target : Path
		constraints: None
		
	Outputs:
		lli_control_request
	'''
	def get_control(self, car_state, target, constraints=None):
		pass


	'''
	Takes in a Pose message as the state.
	
	Returns a dictionary {'x', 'y', 'yaw'} of the same state.
	'''
	def pose_to_xy_yaw(self, state):
		x = state.pose.position.x
		y = state.pose.position.y
		orientation_q = state.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		yaw = euler_from_quaternion(orientation_list)

		return {'x':x, 'y':y, 'yaw':yaw}

	'''
	Inputs:
		car_state: {x, y, yaw} dictionary
		path: [(x,y,k)] tuples of coordinates in order.
		
		returns: (x,y,k) the target point
	'''
	def find_target_index(self, car_state, path):
		pass


	'''
		car_state: {x, y, yaw} dictionary
		target_point: (x,y,k) tuple
		reversing: Bool , whether we are in reverse mode or not
		
		returns: float in range -pi/2, pi/2
	'''
	def calculate_steering_angle(self, car_state, target_point, reversing=False):
		pass

	'''
	angle: float in range -pi/2, pi/2
	
	returns: int in range -100,100
	'''
	def calculate_steering_signal(self, angle):
		pass

	'''
		reference_curvature: float
		
		returns: int in range -100,100
	'''
	def calculate_velocity_signal(self, reference_curvature):
		pass
	

	'''
		path: Path
		
		returns:[(x,y,k)] list of tuples (x, y, curvature)}
	'''
	def path_to_spline_list(self, path):
		pass