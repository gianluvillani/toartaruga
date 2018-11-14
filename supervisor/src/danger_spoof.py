
import rospy
from std_msgs.msg import Float32

if __name__=='__main__':
	rospy.init_node('danger_spoof', anonymous=True)
	d_pub = rospy.Publisher('/obstacles/danger', Float32)
	rate = rospy.Rate(0.5)
	switch = True

	while not rospy.is_shutdown():
		if switch:
			d_pub.publish(0.9)
			switch = False
		else:
			d_pub.publish(0.2)
			switch = True
		rate.sleep()