#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool


class path_memorization:
	
	def __init__(self):
		self.number_of_point=20
		self.leader_past_path= [None]*self.number_of_point
		self.start_publish = 0
		self.start_memorization = False

		self.waypoint_top = rospy.get_param(rospy.get_name() + "/waypoint_topic")
		self.leader_path_top = rospy.get_param(rospy.get_name() + "/leader_path_topic")
		self.memorizing_active_top = rospy.get_param(rospy.get_name() + "/memorizing_active_topic")
		self.start_publish_top = rospy.get_param(rospy.get_name() + "/start_publish_topic")

		self.sub_memorizing_active= rospy.Subscriber(self.memorizing_active_top, Bool, self.start_memorizing)
		self.pub_waypoint = rospy.Publisher(self.leader_path_top, Path)
		self.sub_waypoint = rospy.Subscriber(self.waypoint_top, PoseStamped, self.update_list)
		self.sub_start_publish = rospy.Subscriber(self.start_publish_top, Float32, self.save_start_publish)

	def save_start_publish(self, start_publish_msg):
		self.start_publish = start_publish_msg

	def start_memorizing(self, start_msg):
		self.start_memorization = start_msg.data
		self.number_of_point=20
		self.leader_past_path= [None]*self.number_of_point
	'''
	def save_data(self, x, y):
		out_file = open('/home/nvidia/catkin_ws/src/path_memory/src/leader_path.txt',"a")	
		out_file.write(str(x) + ', ' + str(y) + '\n' )
		out_file.close()
	'''
	def update_list(self, waypoint_msg):
		if self.start_memorization:
			self.new_point = waypoint_msg
			last_waypoint = self.leader_past_path[-1]
			if last_waypoint is not None:
				d = (last_waypoint.pose.position.x - self.new_point.pose.position.x)**2 + (last_waypoint.pose.position.y - self.new_point.pose.position.y)**2
				if d > 0.1**2:

					self.leader_past_path.append(waypoint_msg)
					self.leader_past_path.pop(0)
			elif last_waypoint is None:

				self.leader_past_path.append(waypoint_msg)
	
	def check_if_none(self):
		for item in self.leader_past_path:
        		if item == None:
				return False
    		return True
		
	def publish_leader_path(self):
		path_msg= Path()
		path_msg.header.frame_id = 'qualisys'
		for waypoint in self.leader_past_path:
			path_msg.poses.append(waypoint)
			#self.save_data(waypoint.pose.position.x,waypoint.pose.position.x)

		self.pub_waypoint.publish(path_msg)
			
		
if __name__ == "__main__":
	rospy.init_node('path_memorization')
	rate = rospy.Rate(80)
	my_path = path_memorization()
	while not rospy.is_shutdown():
		if my_path.check_if_none() and my_path.start_publish>0 :
			my_path.publish_leader_path()
		rate.sleep()


