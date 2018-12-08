#!/usr/bin/env python

import math
import rospy
import numpy as np
from control_algorithm import ControlAlgorithm
from spline_interpolation import Spline2D
from low_level_interface.msg import lli_ctrl_request


class PurePursuit(ControlAlgorithm):

	def __init__(self):
		pass

	'''
	Inputs:
		car_state : Pose
		target : Path
		parameters: Dictionary of static values:
			'v': previous/reference speed signal
			'steering': previous/reference steering signal
			'k_lookahead': lookahead constant, scales the lookahead radius
			'l': car length???
		
	Outputs:
		lli_control_request
	'''
	def get_control(self, car_state, target, parameters={'v':50, 'steering':0, 'k_lookahead':1, 'l':2}):
		self.parameters = parameters

		point_path = self.path_to_spline_list(target)
		point_car = self.pose_to_xy_yaw(car_state)

		pure_pursuit_target = self.find_target_point(point_car, point_path)

		angle = self.calculate_steering_angle(point_car, pure_pursuit_target, False)
		steering_signal = self.calculate_steering_signal(angle)
		velocity_signal = self.calculate_velocity_signal(pure_pursuit_target[2])

		ctrl_request_msg = lli_ctrl_request()
		ctrl_request_msg.velocity = int(velocity_signal) #int for safety
		ctrl_request_msg.steering = int(steering_signal) #int for safety

		return ctrl_request_msg


	'''
	Inputs:
		car_state: {x, y, yaw} dictionary
		path: [(x,y,k)] tuples of coordinates in order.
		
		returns: (x,y,k) the target point
	'''
	def find_target_point(self, car_state, path):
		#First, we find the point closest to the car
		best_candidate = 1000000
		best_candidate_index = -1

		for i in range(len(path)):
			current_point = path[i]
			current_distance = (current_point[0] - car_state.get('x'))**2 + (current_point[1] - car_state.get('y'))**2

			if current_distance < best_candidate:
				best_candidate = current_distance
				best_candidate_index = i

		path_length = len(path)
		lookahead_distance = self.parameters.get('k_lookahead') * (1 + 0.01*self.parameters.get('v'))
		current_distance = 0
		current_index = best_candidate_index

		while current_distance < lookahead_distance:
			rospy.logerr("%s , max index is %s", current_index, len(path))
			dL = self.point_distance(path[current_index], path[(current_index+1)% path_length])
			current_distance += dL
			current_index += 1
			current_index %= path_length
			if current_index == best_candidate_index:
				break

		return path[current_index]



	'''
		car_state: {x, y, yaw} dictionary
		target_point: (x,y,k) tuple
		reversing: Bool , whether we are in reverse mode or not
		
		returns: float in range -pi/2, pi/2
	'''
	def calculate_steering_angle(self, car_state, target_point, reversing=False):
		lookahead_radius = self.parameters.get('k_lookahead')* (1 + 0.01*self.parameters.get('v'))

		alpha =  math.atan2(target_point[1] - car_state.get('y'),
		                    target_point[0] - car_state.get('x'))\
		         - car_state.get('yaw')[0]

		if reversing:
			alpha = math.pi - alpha

		delta = math.atan2(2.0 * self.parameters.get('l') * math.sin(alpha) / lookahead_radius, 1.0)
		rospy.logdebug("Delta is %s", delta)

		if abs(delta) > 0.95*math.pi / 4:
			delta = math.copysign(0.95*math.pi / 4, delta)

		return delta


	'''
		reference_curvature: float
		
		returns: int in range -100,100
	'''
	def calculate_velocity_signal(self, reference_curvature):
		k_max = 1/0.2
		v_max = 50
		return v_max*(1-reference_curvature/max(k_max, reference_curvature))

	'''
		path: Path msg
		
		returns:[(x,y,k)] list of tuples (x, y, curvature)}
	'''
	def path_to_spline_list(self, path):

		cx = []
		cy = []
		for pose in path.poses:
			cx.append(pose.pose.position.x)
			cy.append(pose.pose.position.y)
		sp = Spline2D(cx, cy)
		s = np.arange(0, sp.s[-1], 0.01)

		points = []
		for i_s in s:
			ix, iy = sp.calc_position(i_s)
			points.append((ix, iy, sp.calc_curvature(i_s)))

		return points
