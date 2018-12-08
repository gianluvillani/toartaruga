#!/usr/bin/env python

import math
from abc import ABCMeta, abstractmethod
from tf.transformations import euler_from_quaternion
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle
from geometry_msgs.msg import Point


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
    point_a, point_b: geometry_msgs/Point
    
    returns: float: the distance between the points
    '''
    def point_msg_distance(self, point_a, point_b):
        dx = point_a.x - point_b.x
        dy = point_a.y - point_b.y

        return math.sqrt(dx**2 + dy**2)

    '''
    Takes in a Pose message as the state.

    Returns a dictionary {'x', 'y', 'yaw'} of the same state.
    '''

    def pose_to_xy_yaw(self, state):
	if state == None:
		return None
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

    '''
    Inputs:
        circle_obstacles : List of CircleObstacle msgs
        car_state : {x, y, yaw} dictionary.  
    '''
    def transform_obstacles_frame(self, circle_obstacles, car_state):

        circle_obstacles_global = []
        for obstacle in circle_obstacles:
            x_obstacle = obstacle.center.x
            y_obstacle = obstacle.center.y
            alpha = math.atan2(y_obstacle, x_obstacle)
            theta = math.pi + car_state['yaw'][0]
            rho = math.sqrt(x_obstacle ** 2 + y_obstacle ** 2)
            x_obstacle_global = car_state['x'] + rho * math.cos(alpha + theta)
            y_obstacle_global = car_state['y'] + rho * math.sin(alpha + theta)
            obstacle_msg = CircleObstacle()
            obstacle_msg.center = Point()
            obstacle_msg.center.x = x_obstacle_global
            obstacle_msg.center.y = y_obstacle_global
            obstacle_msg.true_radius = obstacle.true_radius
            obstacle_msg.radius = obstacle.radius
            circle_obstacles_global.append(obstacle_msg)

        return circle_obstacles_global

    def path_to_point_list(self, path):
        point_list = []
        for pose_stamped in path:
            p = Point()
            p.x = pose_stamped.pose.point.x
            p.y = pose_stamped.pose.point.y
            point_list.append(p)
        return point_list