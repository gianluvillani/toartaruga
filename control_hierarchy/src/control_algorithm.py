#!/usr/bin/env python

import rospy
from abc import ABCMeta, abstractmethod

# Abstract class for handling several controllers
class ControlAlgorithm:
    __metaclass__ = ABCMeta

    @abstractmethod
    # Different models, different update functions
    def publish_control(self):
        pass

    @abstractmethod
    def parse_state(self):
        pass

    @abstractmethod
    def parse_path(self):
        pass
