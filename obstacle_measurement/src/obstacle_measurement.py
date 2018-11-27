#!/usr/bin/env python

import rospy
import math
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32

class obstacle_measurement:
	def __init__(self):
		self.obstacle_msg = Obstacles()
		self.segment_obstacles = []		
		self.circle_obstacles = []

		# Access rosparams
		self.obstacles_top = rospy.get_param(rospy.get_name() + '/obstacles_topic')
		self.danger_top = rospy.get_param(rospy.get_name() + '/danger_topic')	

		# Publishers and Subscribers
		self.pub_danger = rospy.Publisher(self.danger_top, Float32)
		self.sub_obstacles = rospy.Subscriber(self.obstacles_top, Obstacles, self.save_obstacles)

	def save_obstacles(self, obstacle_msg):
		self.obstacle_msg = obstacle_msg

	def parse_obstacles(self, obstacle_msg):
		self.segment_obstacles = []	
		self.circle_obstacles = []	
		for segment in obstacle_msg.segments:
			self.segment_obstacles.append([(segment.first_point.x, segment.first_point.y), (segment.last_point.x, segment.last_point.y)])
		for circle in obstacle_msg.circles:
			self.circle_obstacles.append((circle.center.x , circle.center.y, circle.radius))	
			
		
	def circle_collision(self, circle_obstacle): # A circle_obstacle is a tuple (x_coordinate, y_coordinate, radius)
		x_obs = circle_obstacle[0]
		y_obs = circle_obstacle[1]
		r_obs = circle_obstacle[2]
		r_car = 0.25
		if x_obs < - math.fabs(0.5*y_obs) and math.fabs(y_obs) <  r_car + r_obs + 0.2 and math.sqrt(x_obs**2 + y_obs**2) < r_car + 0.5:
			print(True)
			return True
		else:
			False
	def publish_danger(self):
		danger_msg = Float32()
		circle_obstacle_found = False
		segment_obstacle_found = False
		for circle in self.circle_obstacles:
			if self.circle_collision(circle):
				circle_obstacle_found = True
		for segment in self.segment_obstacles:
			if self.segment_collision(segment):
				segment_obstacle_found = True
		if circle_obstacle_found or segment_obstacle_found:
			danger_msg.data = 1
		else:		
			danger_msg.data = 0
		self.pub_danger.publish(danger_msg)

		
	def segment_collision(self, segment_obstacle): # A segment_obstacle is a list [(first_point_x, first_point_y), (last_point_x, last_point_y)]
		first_point_x = segment_obstacle[0][0]
		first_point_y = segment_obstacle[0][1]
		last_point_x = segment_obstacle[1][0]
		last_point_y = segment_obstacle[1][1]
		middle_point_x = (first_point_x + last_point_x)/2
		middle_point_y = (first_point_y + last_point_y)/2

		r_car = 0.25
		if middle_point_x < - math.fabs(0.5*middle_point_y) and math.fabs(middle_point_y) <  r_car + 0.2 and ((middle_point_x**2 + middle_point_y**2) < (r_car + 0.5)**2 or (first_point_x**2 + first_point_y**2) < (r_car + 0.5)**2 or (last_point_x**2 + last_point_y**2) < (r_car + 0.5)**2):
			return True
		else:
			False
		

if __name__ == "__main__":
	rospy.init_node('obstacle_measurement', anonymous=True)
	rate = rospy.Rate(10)
	my_obstacle_measurement = obstacle_measurement()
	while not rospy.is_shutdown():
		my_obstacle_measurement.parse_obstacles(my_obstacle_measurement.obstacle_msg)
		my_obstacle_measurement.publish_danger()
		rate.sleep()
	

