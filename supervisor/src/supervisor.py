#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32
import threading

class StateRunning(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['running', 'stop'])
		self.sub_danger = rospy.Subscriber('/obstacles/danger', Float32, self.callback)
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.8
		self.sleep_time = 0.1

	def execute(self, ud):
		self.mutex.acquire()
		if self.danger > self.threshold:
			return 'stop'
		self.mutex.release()
		rospy.sleep(self.sleep_time)
		return 'running'

	def callback(self, msg):
		self.mutex.acquire()
		self.danger = msg
		self.mutex.release()

class StateStopped(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['stopped', 'start'])
		self.sub_danger = rospy.Subscriber('/obstacles/danger', Float32, self.callback)
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.1

	def execute(self, ud):
		self.mutex.acquire()
		if self.danger < self.threshold:
			return 'start'
		self.mutex.release()
		rospy.sleep(self.sleep_time)
		return 'stopped'

	def callback(self, msg):
		self.mutex.acquire()
		self.danger = msg
		self.mutex.release()


if __name__ == "__main__":
	rospy.init_node('supervisor', anonymous=True)
	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('StateRunning', StateRunning(), transitions={'running':'StateRunning', 'stop':'StateStopped'})
		smach.StateMachine.add('StateStopped', StateStopped(), transitions={'stopped':'StateStopped', 'start':'StateRunning'})

	outcome = sm.execute()

