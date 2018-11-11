#!/usr/bin/env python

import tf
import tf2_ros
import rospy
from math import pi

import geometry_msgs.msg
from nav_msgs.msg import Odometry

class transform_broadcaster:
	def __init__(self):
		self.lidar_frame_id = "map"
		self.global_frame_id = "qualisys"
		self.broadcaster = tf2_ros.TransformBroadcaster()
        	self.t = geometry_msgs.msg.TransformStamped()
		self.sub_odom = rospy.Subscriber('/SVEA2/odom', Odometry, self.update_transform)
		
	def update_transform(self, odom_msg):
		"""
		Updates the trasnforms, given the vehicle state
		:param odom_msg:
		:return:
		"""
		self.t.header.stamp = rospy.Time.now()
		self.t.header.frame_id = self.global_frame_id
		self.t.child_frame_id = self.lidar_frame_id
		self.t.transform.translation.x = odom_msg.pose.pose.position.x
		self.t.transform.translation.y = odom_msg.pose.pose.position.y
		self.t.transform.translation.z = 0
	
                orientation_q = odom_msg.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		# The x and y direction of the lidar is -x and -y of the vehicle.                
		lidar_yaw = tf.transformations.euler_from_quaternion(orientation_list)[2] + pi
		q = tf.transformations.quaternion_from_euler(0, 0, lidar_yaw)
		self.t.transform.rotation.x = q[0]
		self.t.transform.rotation.y = q[1]
		self.t.transform.rotation.z = q[2]
		self.t.transform.rotation.w = q[3]

		self.broadcaster.sendTransform(self.t)

if __name__ == "__main__":
	rospy.init_node('transform_broadcaster', anonymous=True)
	tf_broadcaster = transform_broadcaster()
	rospy.spin()

