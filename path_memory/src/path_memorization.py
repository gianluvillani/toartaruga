#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class path_memorization:
	
	def __init__(self):
		self.number_of_point=200
		self.leader_past_path= [None]*self.number_of_point
		#self.waypoint_top = rospy.get_param(rospy.get_name() + "/waypoint_topic")
		self.waypoint_top = '/leader/pose'
		self.sub_waypoint = rospy.Subscriber(self.waypoint_top, PoseStamped, self.update_list)
		self.pub_waypoint = rospy.Publisher('leader_path', Path)
	

	def update_list(self, waypoint_msg):
		self.new_point = waypoint_msg
		self.leader_past_path.append(waypoint_msg)
		self.leader_past_path.pop(0)
	
	def check_if_none(self):
		for item in self.leader_past_path:
        		if item == None:
            			return False
    		return True
		
	def publish_leader_path(self):
		path_msg= Path()
		path_msg.header.frame_id = 'qualisys'
		if self.check_if_none:
			for waypoint in self.leader_past_path:
				path_msg.poses.append(waypoint)
			self.pub_waypoint.publish(path_msg)
		else: 
			pass
			
		
if __name__ == "__main__":
	rospy.init_node('leader_path_recording')	
	rate = rospy.Rate(80)
	my_path = path_memorization()
	while not rospy.is_shutdown():
		my_path.publish_leader_path()
		rate.sleep()


