#!/usr/bin/env python

import tf
import tf2_ros
import rospy
import operator
import math
import numpy as np
import geometry_msgs.msg
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import nav_msgs.msg 
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from low_level_interface.msg import lli_ctrl_request
#TODO: Use low_level_interface on NVIDIA
from geometry_msgs.msg import Twist
from spline_interpolation import Spline2D
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import copy
import random
import path_utils
import matplotlib.pyplot as plt
import timeit
import path_utils
#import plot_utils
from spline_interpolation import Spline2D
DEBUG = True


def log(s):
    if DEBUG:
        print(s)



def get_course():
    t = np.arange(0, math.pi * 2, 0.01)
    x = np.cos(t)
    y = np.sin(t)
    return path_utils.Path(list(x), list(y))


class DynamicPlanner:
    def __init__(self, safety_distance=0.1):
        self.state_car = PoseStamped()
        self.obstacle_msg = Obstacles()
	self.obstacle_handler = path_utils.ObstacleHandler()
        self.x_car = 0
        self.y_car = 0
        self.yaw_car = 0
        self.car_index = 0
        self. min_dist_back_on_path = 0.1
	self.reference_msg = nav_msgs.msg.Path()
        self.safety_distance = safety_distance
        self.circle_obstacles = []

        self.reference_path = get_course()

        self.previous_path = copy.deepcopy(self.reference_path)

        self.path_available = False
        self.state_available = False
        self.obstacles_available = False
        self.reset_obstacles = False

        self.min_dist_obstacle_crossing = 0.9
        self.ang_on_path = math.pi / 5

        # Access rosparams
        #self.obstacles_top = rospy.get_param(rospy.get_name() + '/obstacles_topic')
        #self.car_pose_top = rospy.get_param(rospy.get_name() + "/car_pose_topic")
        #self.path_top = rospy.get_param(rospy.get_name() + "/path_topic")
        #self.replanner_path_top = rospy.get_param(rospy.get_name() + "/replanner_path_topic")

        # Publishers/Subscriber
        self.sub_obstacles = rospy.Subscriber('/obstacles', Obstacles, self.save_obstacles)
        self.sub_car_pose = rospy.Subscriber('/SVEA2/pose', PoseStamped, self.save_state)
        self.sub_path = rospy.Subscriber('/SVEA2/path', nav_msgs.msg.Path, self.save_path)
        self.pub_path = rospy.Publisher('/sample_path', nav_msgs.msg.Path)
	topic = 'visualization_marker_array'
	self.pub_obstacle = rospy.Publisher(topic, MarkerArray)

        #self.pub_marker = rospy.Publisher('/marker', PoseStamped)

    def save_path(self, path_msg):
	self.reference_msg = path_msg
        self.path_available = True

    def save_state(self, state_msg):
	#print('saving pose')        
	self.state_car = state_msg
        self.state_available = True
    
    def parse_path(self):
	cx = []
	cy = []
	
	for pose in self.reference_msg.poses:
		cx.append(pose.pose.position.x)
		cy.append(pose.pose.position.y)
	self.reference_path = path_utils.Path(list(cx), list(cy))

    def parse_state(self):
	state_msg = self.state_car
	self.x_car = state_msg.pose.position.x
	self.y_car = state_msg.pose.position.y
	orientation_q = state_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	self.yaw_car = euler_from_quaternion(orientation_list)[2]

    def save_obstacles(self, obstacle_msg):
	#print('Saving '+str(len(obstacle_msg.circles))+' obstacles')
        self.obstacle_msg = obstacle_msg
        self.obstacles_available = True

    def transform2global(self, x, y):
        alpha = math.atan2(y, x)
        theta = math.pi + self.yaw_car
        rho = math.sqrt(x ** 2 + y ** 2)
        x_global = (self.x_car + rho * math.cos(alpha + theta))
        y_global = (self.y_car + rho * math.sin(alpha + theta))
        return x_global, y_global
        #return x, y

    def on_path(self, obstacle):
        obstacle_on_path = False
        for x, y in zip(self.previous_path.cx, self.previous_path.cy):
            if math.sqrt((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) < \
                    obstacle.r + self.safety_distance:
                obstacle_on_path = True
                break
        return obstacle_on_path

    def make_marker(self, obstacle):
	marker = Marker()
   	marker.header.frame_id = "qualisys"
   	marker.type = marker.SPHERE
   	marker.action = marker.ADD
   	marker.scale.x = 0.1
   	marker.scale.y = 0.1
   	marker.scale.z = 0.1
   	marker.color.a = 1.0
   	marker.color.r = 1.0
   	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = obstacle.x
	marker.pose.position.y = obstacle.y 
	marker.pose.position.z = 0 
	return marker

    def parse_obstacles(self):
	markerArray = MarkerArray()
	obstacle_msg = self.obstacle_msg
        car_index = self.reference_path.get_closest_index_on_path(self.x_car, self.y_car)
        self.car_index = car_index
	print('Received '+ str(len(obstacle_msg.circles)) + ' obstacles')
	obstacles_on_path = []
        for circle in obstacle_msg.circles:
            x_obstacle_global, y_obstacle_global = self.transform2global(circle.center.x, circle.center.y)
            obstacle = path_utils.Obstacle(x_obstacle_global, y_obstacle_global, circle.radius)
            index_on_path = self.reference_path.get_closest_index_on_path(obstacle.x, obstacle.y)
	    if math.hypot(x_obstacle_global-self.reference_path.cx[index_on_path], y_obstacle_global-self.reference_path.cy[index_on_path]) < 1:
	        obstacles_on_path.append(obstacle)
	
	self.obstacle_handler.check_new_obstacles(obstacles_on_path)
	sorted_obstacles = []
	for obstacle in self.obstacle_handler.obstacle_list:
            index_on_path = self.reference_path.get_closest_index_on_path(obstacle.x, obstacle.y)
	    gradient_on_path = self.reference_path.compute_direction(index_on_path)
	    obstacle.compute_wp(gradient_on_path, self.safety_distance)
	    obstacle.index_on_path = (index_on_path - car_index) % self.reference_path.get_length()
	    if len(sorted_obstacles) == 0:
	        sorted_obstacles.append(obstacle)
	    else:
	        for index, sorted_obstacle in enumerate(sorted_obstacles):
	            if (obstacle.index_on_path  - car_index) % self.reference_path.get_length() <= (sorted_obstacle.index_on_path - car_index) % self.reference_path.get_length() :
	                sorted_obstacles.insert(index, obstacle)
	                break
	        if obstacle.index_on_path > sorted_obstacles[-1].index_on_path:
	            sorted_obstacles.append(obstacle)
                    marker = self.make_marker(obstacle)
                    markerArray.markers.append(marker)

	self.obstacle_handler.obstacle_list = sorted_obstacles
	id = 0
  	for m in markerArray.markers:
       		m.id = id
        	id += 1	
		print(id)
	self.pub_obstacle.publish(markerArray)
	print('Saved '+ str(len(self.circle_obstacles)) + ' obstacles')

    def crossing(self, wp1, wp2):
        delta_x = wp1.obstacle.x - wp2.obstacle.x
        delta_y = wp1.obstacle.y - wp2.obstacle.y
        if delta_x != 0:
            m = delta_y / delta_x
            q = wp1.obstacle.y - m * wp1.obstacle.x
            if (wp1.y >= m * wp1.x + q) and (wp2.y >= m * wp2.x + q):
                return False
            if (wp1.y <= m * wp1.x + q) and (wp2.y <= m * wp2.x + q):
                return False
            return True
        else:
            if (wp1.x - wp1.obstacle.x) * (wp2.x - wp2.obstacle.x) >= 0:
                return False
        return True

    def check_feasible_path(self, current_wp, next_wp):
        if next_wp.obstacle is None or current_wp.obstacle is None:
            return True
        distance_obstacle = math.hypot(current_wp.obstacle.x - next_wp.obstacle.x,
                                       current_wp.obstacle.y - next_wp.obstacle.y)
        clean_distance = distance_obstacle - (current_wp.obstacle.r + next_wp.obstacle.r)
        if self.crossing(current_wp, next_wp) and clean_distance < self.min_dist_obstacle_crossing:
            return False
        else:
            return True

    def backward_recursion(self):
        fake_first_obstacle = path_utils.Obstacle(x=self.x_car,
                                                  y=self.y_car,
                                                  r=0)
        index_on_path = self.previous_path.get_closest_index_on_path(fake_first_obstacle.x, fake_first_obstacle.y)
        fake_first_obstacle.index_on_path = (index_on_path - self.car_index) % self.previous_path.get_length()
        fake_first_obstacle.waypoints = [path_utils.WayPoint(self.x_car,
                                                             self.y_car,
                                                             None)]
	self.obstacle_handler.obstacle_list.append(fake_first_obstacle)	
	obstacles = self.obstacle_handler.obstacle_list
        for wp in obstacles[-1].waypoints:
            wp.end = True
            wp.feasible = True
            wp.set_cost_to_go(0)

        for i in range(len(obstacles)-2, -1, -1):
            current_obs = obstacles[i]
            next_obs = obstacles[i+1]
            next_wps = next_obs.waypoints
            log('CURRENT OBSTACLE: ' + str(current_obs.x) + ", " + str(current_obs.y))
            # TODO: If clear distance is enough: indexes on path / ds
            distance_on_path = ((next_obs.index_on_path - current_obs.index_on_path) % self.previous_path.get_length()) * self.previous_path.ds
            dist_last = (next_obs.r + next_obs.num_wp * self.safety_distance) / math.cos(self.ang_on_path)
            dist_first = (current_obs.r + current_obs.num_wp * self.safety_distance) / math.cos(self.ang_on_path)
            clean_distance = distance_on_path - (dist_last + dist_first)
            if clean_distance > self.min_dist_back_on_path:
                next_wps = []
                additional_wp = int(clean_distance / (self.previous_path.ds * 2))
                shift_index = int((clean_distance/additional_wp / self.previous_path.ds))
                bias_index = int(dist_last/self.previous_path.ds)
                old_wp_list = next_obs.waypoints
                for i in range(additional_wp):
                    current_wp_index = (next_obs.index_on_path - (shift_index * (i+1) + bias_index) + self.car_index) % self.previous_path.get_length()
                    new_wp = path_utils.WayPoint(self.previous_path.cx[current_wp_index], self.previous_path.cy[current_wp_index], None)
                    new_wp.feasible = True
                    self.find_optimal_next(new_wp, old_wp_list)
                    # new_wp.next_wp = old_wp
                    old_wp_list = [new_wp]
                next_wps = [new_wp]

            for wp0 in current_obs.waypoints:
                log('Current waypoint: ' + str(wp0.x) + ", " + str(wp0.y))
                if wp0.obstacle is not None:
                    log('   On obstacle: ' + str(wp0.obstacle.x) + ", " + str(wp0.obstacle.y))
                feasible_waypoints = []
                for wp1 in next_wps:
                    if wp1.feasible:
                        if self.check_feasible_path(wp0, wp1):
                            feasible_waypoints.append(wp1)
                if len(feasible_waypoints) == 0:
                    wp0.feasible = False
                else:
                    wp0.feasible = True
                    self.find_optimal_next(current_wp=wp0, next_feasible_wps=feasible_waypoints)
	print('BACKWARD RECURSION ENDED')

    def find_optimal_next(self, current_wp, next_feasible_wps):
        index_on_path = self.previous_path.get_closest_index_on_path(current_wp.x, current_wp.y)
        reward_now = math.hypot(current_wp.x - self.previous_path.cx[index_on_path],
                                current_wp.y - self.previous_path.cy[index_on_path])
        best_wp = next_feasible_wps[0]
        min_cost = 100

        for w in next_feasible_wps:
            log('       Possible future waypoint: ' + str(current_wp.x) + ", " + str(current_wp.y))
            if w.obstacle is not None:
                log('       On obstacle: ' + str(w.obstacle.x) + ", " + str(w.obstacle.y))
                log('       Future cost:' + str(w.get_cost_to_go()))
            if w.get_cost_to_go() < min_cost:
                min_cost = w.get_cost_to_go()
                best_wp = w
        log('           Best future cost: ' + str(min_cost))
        log('           With point: ' + str(best_wp.x) + ", " + str(best_wp.y))
        current_wp.set_cost_to_go(min_cost + reward_now)
        current_wp.next_wp = best_wp
        return best_wp


class FakeObstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.radius = r

def coordinates_to_msg(sx, sy, sk):
    path_msg = nav_msgs.msg.Path()
    z_init = 0
    for x, y, k in zip(sx, sy, sk):
        pose = PoseStamped()
        pose.header.frame_id = 'qualisys'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z_init
        z_init += 0.01
        path_msg.poses.append(pose)
    path_msg.header.frame_id = 'qualisys'
    return path_msg


if __name__ == "__main__":
    rospy.init_node('path_publisher', anonymous=True)
    planner = DynamicPlanner(safety_distance=0.3)
    planner.x_car = 1
    planner.y_car = 0
    planner.yaw_car = math.pi/2
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
	#try:
	#print('Try')
 
	planner.parse_obstacles()
	print(planner.obstacle_handler.new_obstacles)
	print(len(planner.obstacle_handler.obstacle_list))
	if planner.obstacle_handler.new_obstacles and len(planner.obstacle_handler.obstacle_list)>1: 	
		planner.parse_state()
		planner.parse_path()  
		planner.backward_recursion()
	    	min_cost = 200
	    	start_wp = planner.obstacle_handler.obstacle_list[0].waypoints[0]
	    	for wp in planner.obstacle_handler.obstacle_list[0].waypoints:
			tx = wp.x
	    		ty = wp.y
			alpha = math.atan2(ty - planner.y_car, tx - planner.x_car) - planner.yaw_car
			Lf = math.hypot(planner.x_car - wp.x, planner.y_car - wp.y)
			delta = math.atan2(2.0 * 0.2 * math.sin(alpha) / Lf, 1.0)
			if abs(delta) < min_cost:
				min_cost = abs(delta)
				start_wp = wp

	    	next_wp = start_wp
	    	to_be_splined_x = [start_wp.x]
	    	to_be_splined_y = [start_wp.y]
	    	end = False
	    	while not end:
			to_be_splined_x.append(next_wp.x)
			to_be_splined_y.append(next_wp.y)
			end = next_wp.end
			if not next_wp.end:
	    			next_wp = next_wp.next_wp

		print('Splinin ' + str(len(to_be_splined_x)))
		print(to_be_splined_x[1:-1])
		print(to_be_splined_y[1:-1])
	    	spl = Spline2D(to_be_splined_x[2:-1], to_be_splined_y[2:-1])
	    	s = np.arange(0, spl.s[-1], 0.1)
	    	sx = []
	    	sy = []
	    	for i_s in s:
			ix, iy = spl.calc_position(i_s)
			sx.append(ix)
			sy.append(iy)
		path_msg = coordinates_to_msg(sx, sy, sx)
		print('Publishing msg')
		planner.pub_path.publish(path_msg)
		planner.reset_obstacles = True
	#except:
	#	print('continue')
	#	pass
        rate.sleep()
    

