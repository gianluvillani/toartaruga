#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import threading

class StateIdle(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['stopped', 'start'])
		self.sub_danger = rospy.Subscriber('/obstacles/danger', Float32, self.danger_callback)
		self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.path_callback)
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.8
		self.sleep_time = 0.1
		self.path = None

	def execute(self, ud):
		while True:
			self.mutex.acquire()
			if self.path != None:
				if self.danger > self.threshold:
					self.mutex.release()
					return 'stopped'
				else:
					self.mutex.release()
					return 'start'
			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def path_callback(self, msg):
		self.mutex.acquire()
		self.path = msg
		self.mutex.release()

	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg
		self.mutex.release()

class StateRunning(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['running', 'stop'])
		self.sub_danger = rospy.Subscriber('/obstacles/danger', Float32, self.callback)
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.8
		self.sleep_time = 0.1
		#service call: Start tracking


	def execute(self, ud):
		while True:
			self.mutex.acquire()
			if self.danger > self.threshold:
				self.mutex.release()
				return 'stop'
			self.mutex.release()
			rospy.sleep(self.sleep_time)

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
		# service call: stop tracking

	def execute(self, ud):
		while True:
			self.mutex.acquire()
			if self.danger < self.threshold:
				return 'start'
			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def callback(self, msg):
		self.mutex.acquire()
		self.danger = msg
		self.mutex.release()


if __name__ == "__main__":
	rospy.init_node('supervisor', anonymous=True)
	sm = smach.StateMachine(outcomes=[])
	with sm:
		smach.StateMachine.add('StateIdle', StateIdle(),
		                       transitions={'start': 'StateRunning', 'stopped': 'StateStopped'})
		smach.StateMachine.add('StateRunning', StateRunning(),
		                       transitions={'running':'StateRunning', 'stop':'StateStopped'})
		smach.StateMachine.add('StateStopped', StateStopped(),
		                       transitions={'stopped':'StateStopped', 'start':'StateRunning'})
	outcome = sm.execute()

