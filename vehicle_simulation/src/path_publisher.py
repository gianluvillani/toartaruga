#! /usr/bin/env python
import os
import rospy
import math
#import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path, Odometry
from low_level_interface.msg import lli_ctrl_request
from tf.transformations import euler_from_quaternion 
from geometry_msgs.msg import PoseStamped



def parse_path_file(filepath = '/home/nvidia/catkin_ws/src/vehicle_simulation/src/coordinates.txt'):
	print filepath
	cx = []
	cy = []
	with open(filepath, 'r') as f:
		for line in f:
			parsed_line = line.rstrip('\n').split(',')
			cx.append(float(parsed_line[0]))
			cy.append(float(parsed_line[1]))
	return cx, cy
	
def coordinates_to_msg(cx, cy):
	path_msg = Path()
	for x, y in zip(cx, cy):
		pose = PoseStamped()
		pose.header.frame_id = 'odom'
		pose.pose.position.x = x
		pose.pose.position.y = y	
		path_msg.poses.append(pose)
	path_msg.header.frame_id = 'odom'
	return path_msg

if __name__ == "__main__":
	rospy.init_node('path_publisher', anonymous=True)	
	pub = rospy.Publisher('/SVEA2/path', Path)
	cx, cy = parse_path_file()
	path_msg = coordinates_to_msg(cx, cy)    
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		pub.publish(path_msg)
		rate.sleep()
		

