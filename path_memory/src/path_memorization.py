#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class path_memorization:
	
	def __init__(self):
		self.number_of_point=200
		self.leader_past_path= [None]*self.number_of_point


		self.waypoint_top = rospy.get_param(rospy.get_name() + "/waypoint_topic")
		self.leader_path_top = rospy.get_param(rospy.get_name() + "/leader_path_topic")
	
		self.sub_waypoint = rospy.Subscriber(self.waypoint_top, PoseStamped, self.update_list)
		self.pub_waypoint = rospy.Publisher(self.leader_path_top, Path)
		self.count = 0
	

	def update_list(self, waypoint_msg):
		self.new_point = waypoint_msg
		self.count += 1
		#if self.count %5 == 0 :
		self.leader_past_path.append(waypoint_msg)
		self.leader_past_path.pop(0)
		rospy.logerr ('len= %s', len(self.leader_past_path))
	
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
		self.pub_waypoint.publish(path_msg)
			
		
if __name__ == "__main__":
	rospy.init_node('path_memorization')	
	rate = rospy.Rate(100)
	my_path = path_memorization()
	while not rospy.is_shutdown():
		if my_path.check_if_none():
			my_path.publish_leader_path()
		rate.sleep()


