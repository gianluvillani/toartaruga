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
		# Publishers and Subscribers
		#self.sub_pose = rospy.Subscriber('/SVEA2/pose', PoseStamped, self.parse_state)
		#self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.parse_path)
		self.pub_danger = rospy.Publisher('/danger', Float32)
		self.sub_obstacles = rospy.Subscriber('/obstacles', Obstacles, self.save_obstacles)
	"""
	def parse_state(self, pose_msg):
                self.x_car = pose_msg.pose.position.x
                self.y_car = pose_msg.pose.position.y
                orientation_q = pose_msg.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                self.yaw_car = euler_from_quaternion(orientation_list)[2]
	def parse_path(self, path_msg):			
		self.path_x_list = []
		self.path_y_list = []
		for pose in path_msg.poses:
			self.path_x_list.append(pose.pose.position.x)
			self.path_y_list.append(pose.pose.position.y)
	"""
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
		obstacle_found = False
		for circle in self.circle_obstacles:
			if self.circle_collision(circle):
				obstacle_found = True
		if obstacle_found:
			danger_msg.data = 1
		else:		
			danger_msg.data = 0
		self.pub_danger.publish(danger_msg)

		

	def segment_distance(self, segment_obstacle): # A segment_obstacle is a list [(first_point_x, first_point_y), (last_point_x, last_point_y)]
		x_obs = circle_obstacle[0]
		y_obs = circle_obstacle[1]
		r_obs = circle_obstacle[2]
		r_car = 0.25
		if x_obs < - math.fabs(0.5*y_obs) and math.fabs(y_obs) <  r_car + r_obs + 0.1:
			return math.sqrt(x_obs**2 + y_obs**2)
		

if __name__ == "__main__":
	rospy.init_node('obstacle_measurement', anonymous=True)
	rate = rospy.Rate(10)
	my_obstacle_measurement = obstacle_measurement()
	while not rospy.is_shutdown():
		my_obstacle_measurement.parse_obstacles(my_obstacle_measurement.obstacle_msg)
		my_obstacle_measurement.publish_danger()
		rate.sleep()
	

