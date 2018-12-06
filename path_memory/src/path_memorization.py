#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32


class path_memorization:
	
	def __init__(self):
		self.number_of_point=250
		self.leader_past_path= [None]*self.number_of_point
		self.start_publish = 0


		self.waypoint_top = rospy.get_param(rospy.get_name() + "/waypoint_topic")
		self.waypoint_top = ("/SVEA1/pose")
		self.leader_path_top = rospy.get_param(rospy.get_name() + "/leader_path_topic")
		
	
		self.sub_waypoint = rospy.Subscriber(self.waypoint_top, PoseStamped, self.update_list)
		self.pub_waypoint = rospy.Publisher(self.leader_path_top, Path)
		self.sub_start_publish = rospy.Subscriber('/start_publish_top', Float32, self.save_start_publish)
		self.count = 0

	def save_start_publish(self, start_publish_msg):
		self.start_publish = start_publish_msg

	def update_list(self, waypoint_msg):
		self.new_point = waypoint_msg
		self.count += 1
		#if self.count %5 == 0 :
		self.leader_past_path.append(waypoint_msg)
		self.leader_past_path.pop(0)
		#rospy.logerr ('len= %s', len(self.leader_past_path))
	
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
		rospy.logerr ('check if none= %s', my_path.check_if_none())
		if my_path.check_if_none() and my_path.start_publish>0 :
			rospy.logerr ('you are in the if and check if none is= %s', my_path.check_if_none())
			my_path.publish_leader_path()
			my_path.start_publish=0
		rate.sleep()


