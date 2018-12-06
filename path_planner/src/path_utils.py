import numpy as np
import math
from spline_interpolation import Spline2D
import copy

class WayPoint:
    def __init__(self, x, y, obstacle):
        self.x = x
        self.y = y
        self.next_wp = None
        self.feasible = True
        self.end = False
        self.obstacle = obstacle

        self.cost_set = False
        self.cost_to_go = None

    def get_next_wp(self):
        if self.feasible:
            return self.next_wp
        else:
            return None

    def set_cost_to_go(self, cost):
        self.cost_set = True
        self.cost_to_go = cost

    def get_cost_to_go(self):
        if self.obstacle is None:
            return 0
        else:
            assert self.cost_to_go is not None, "cost_to_go not set"
            return self.cost_to_go


class Obstacle:
    def __init__(self, x, y, r, num_wp=5):
        self.x = x
        self.y = y
        self.r = r
	self.discretization = 0.02

        self.index_on_path = 0
        self.wp1 = None
        self.wp2 = None
        self.num_wp = num_wp
        self.waypoints = []

    def compute_wp(self, gradient, safety_distance):
        waypoint_direction = np.zeros(2)
        waypoint_direction[0] = -gradient[1]
        waypoint_direction[1] = gradient[0]
        waypoint_direction = waypoint_direction / np.linalg.norm(waypoint_direction, ord=2)
        current_position = np.array([self.x, self.y])
        for i in range(self.num_wp):
            wp1 = current_position + waypoint_direction * (self.r + safety_distance + self.discretization * (i + 1))
            wp2 = current_position - waypoint_direction * (self.r + safety_distance + self.discretization * (i + 1))
            wp1 = WayPoint(wp1[0], wp1[1], self)
            wp2 = WayPoint(wp2[0], wp2[1], self)
            self.waypoints.append(wp1)
            self.waypoints.append(wp2)


class Path:
    def __init__(self, cx, cy, ds=0.05):
        # spline and save.
        spl = Spline2D(cx, cy)
        s = np.arange(0, spl.s[-1], ds)
        sx = []
        sy = []
        for i_s in s:
            ix, iy = spl.calc_position(i_s)
            ik = spl.calc_curvature(i_s)
            sx.append(ix)
            sy.append(iy)
        self.cx = sx
        self.cy = sy
        self.s = s
        self.ds = ds

    def get_length(self):
        return len(self.cx)

    def compute_direction(self, index):
        """
        :param index: index in which we would like to get the direction of the path
        :return: unit vector in that direction
        """
        path_direction = np.array([self.cx[(index + 1) % len(self.cx)] - self.cx[index], self.cy[(index + 1) % len(self.cy)] - self.cy[index]])
        waypoint_direction = np.zeros(2)
        waypoint_direction[0] = path_direction[0]
        waypoint_direction[1] = path_direction[1]
        waypoint_direction = waypoint_direction/np.linalg.norm(waypoint_direction, ord=2)
        return waypoint_direction

    def get_closest_index_on_path(self, x, y):
        """
        :param x:
        :param y:
        :return: given (x, y), find index of closest point on path
        """
        dx = [x - icx for icx in self.cx]
        dy = [y - icy for icy in self.cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

        return d.index(min(d))

    def update_path(self):
        pass


class ObstacleHandler:
    def __init__(self):
        self.obstacle_list = None
        self.counter = 0
        self.num_update = 5
        self.threshold_dist = 0.15
        self.initialized = False
        self.new_obstacles = True

    def check_new_obstacles(self, new_obstacles):
        if not self.initialized:
            self.obstacle_list = copy.deepcopy(new_obstacles)
            return
        for new in new_obstacles:
            for old in self.obstacle_list:
                if math.hypot(old.x - new.x, old.y - new.y) > self.threshold_dist:
                    self.obstacle_list = copy.deepcopy(new_obstacles)
                    self.new_obstacles = True
                    return
                else:
                    old.x = (old.x + new.x)/2
                    old.y = (old.y + new.y)/2
                    self.new_obstacles = False

    def check_init(self):
        return self.initialized
