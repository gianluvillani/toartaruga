#!/usr/bin/env python

import rospy
from abc import ABCMeta, abstractmethod

# Abstract class for handling several controllers
class ControlAlgorithm:
    __metaclass__ = ABCMeta

    '''
        car_state is always an Odometry message, containing the current position of the car
        target is whatever the current algorithm needs as input:
            - a Path for pure pursuit
            - a Waypoint for PID
        constraints is future-proofing for any obstacles that the ALGORITHM needs. Will be given as a list of Obstacles.
            
    '''
    @abstractmethod
    def get_control(self, car_state, target, constraints=None):
        pass