#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from low_level_interface.msg import lli_ctrl_request

def callback(vel_msg):
	linear_vel = vel_msg.linear.x
	angular_vel = vel_msg.angular.z
	ctrl_request_msg = lli_ctrl_request()
	ctrl_request_msg.velocity = int(linear_vel)
	ctrl_request_msg.steering = int(angular_vel)
	pub.publish(ctrl_request_msg)

rospy.init_node('teleop_transcriber')
sub = rospy.Subscriber('/key_vel', Twist, callback)
pub = rospy.Publisher('/lli/ctrl_request',lli_ctrl_request)
rospy.spin()
