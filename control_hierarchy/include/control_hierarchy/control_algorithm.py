#!/usr/bin/env python

import math
from abc import ABCMeta, abstractmethod
from tf.transformations import euler_from_quaternion


# Abstract class for handling several controllers
class ControlAlgorithm:
    __metaclass__ = ABCMeta

    '''
        car_state is always an Odometry message, containing the current position of the car
        target is whatever the current algorithm needs as input:
            - a Path for pure pursuit
            - a Waypoint for PID
        parameteters is a dictionary tailored to the algorithm.
            
    '''
    @abstractmethod
    def get_control(self, car_state, target, parameters=None):
        pass

    '''
    point_a, point_b: (x,y,k) float tuples
    
    returns: float: the distance between the points
    '''
    def point_distance(self, point_a, point_b):
        dx = point_a[0]-point_b[0]
        dy = point_a[1]-point_b[1]

        return math.sqrt(dx**2 + dy**2)

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

        return {'x': x, 'y': y, 'yaw': yaw}

    '''
    angle: float in range -pi/4, pi/4

    returns: int in range -100,100
    '''

    def calculate_steering_signal(self, angle):
        return angle * (400 / math.pi)

