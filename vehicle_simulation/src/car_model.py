from abc import ABCMeta, abstractmethod
from math import tan, atan, cos, sin, copysign, pi


# Abstract class for handling several vehicles models
class Model:
    __metaclass__ = ABCMeta
    @abstractmethod
    # Different models, different update functions
    def update_state(self):
        pass

    @abstractmethod
    def get_state(self):
        pass


# Kinematic bicycle model, velocity and steering inputs
# https://github.com/MPC-Berkeley/barc/wiki/Car-Model
class KinematicBicycle(Model):

    def __init__(self, l_f=0.1, l_r=0.1, dt=0.5, x_0=0, y_0=0, yaw_0=0, v_0=0, max_delta=pi/4):
        """
        :param l_r: distance of the center of mass from rear wheels
        :param l_f: distance of the center of mass from front wheels
        :param x_0: starting x|y|yaw|v states
        :param dt: discretization time step
        :param max_delta: maximum steering angle
        """
        self.l_f = l_f
        self.l_r = l_r
        self.dt = dt
        self.max_delta = max_delta

        self.x = x_0
        self.y = y_0
        self.yaw = yaw_0
        self.v = v_0

    def update_state(self, v, delta):
        """
        :param v: input velocity
        :param delta: steering angle
        """
        # steering saturation
        if abs(delta)>self.max_delta:
            delta = copysign(self.max_delta, delta)
        # state update
        beta = atan(self.l_r / (self.l_r + self.l_f) * tan(delta))
        self.x = self.x + self.v * cos(self.yaw + beta) * self.dt
        self.y = self.y + self.v * sin(self.yaw + beta) * self.dt
        self.yaw = self.yaw + self.v/self.l_f * sin(beta) * self.dt
        self.v = v

    def get_state(self):
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw}
