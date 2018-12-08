#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Path
import threading
from control_hierarchy.dispatcher import Dispatcher


class StateIdle(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['stop', 'run', 'follow'])
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.path_callback)
		self.sub_follow = rospy.Subscriber('/another_car', Bool, self.other_car_callback)

		self.pub_start_stop_controller = rospy.Publisher('/start_stop_controller', Bool)
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.8
		self.sleep_time = 0.1
		self.path = None
		self.another_car = False

	def execute(self, ud):
		while not rospy.is_shutdown():
			self.mutex.acquire()
			if self.another_car:
				if self.danger > self.threshold:
					self.mutex.release()
					return 'stop'
				else:
					self.mutex.release()
					return 'follow'

			if self.path != None:
				if self.danger > self.threshold:
					self.mutex.release()
					return 'stop'
				else:
					self.mutex.release()
					return 'run'
				
			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def path_callback(self, msg):
		self.mutex.acquire()
		self.path = True
		self.mutex.release()

	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.another_car = msg.data
		self.mutex.release()

class StateRunning(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['run', 'stop', 'follow'])
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_follow = rospy.Subscriber('/another_car', Bool, self.other_car_callback)
		self.dispatcher = dispatcher
		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.8
		self.sleep_time = 0.02
		self.other_car = False

	def execute(self, ud):

		while not rospy.is_shutdown():
			self.mutex.acquire()
			self.dispatcher.publish_control()
			if self.danger > self.threshold:
				self.mutex.release()
				return 'stop'

			if self.other_car:
				self.mutex.release()
				return 'follow'

			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()


class StateStopped(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['stop', 'run', 'follow'])

		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_other_car = rospy.Subscriber('/another_car', Bool, self.other_car_callback)
		self.mutex = threading.Lock()

		self.danger = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.1
		self.other_car = False
		# service call: stop tracking

	def execute(self, ud):

		while not rospy.is_shutdown():
			self.mutex.acquire()

			if self.danger < self.threshold:
				self.mutex.release()
				return 'run'

			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()


class StateFollowing(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['stop', 'idle', 'follow'])
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_other_car = rospy.Subscriber('/another_car', Bool, self.other_car_callback)
		self.dispatcher = dispatcher

		self.mutex = threading.Lock()
		self.danger = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.02
		self.other_car = True

	def execute(self, ud):
		while not rospy.is_shutdown():
			self.mutex.acquire()
			self.dispatcher.publish_control()
			if self.danger > self.threshold:
				self.mutex.release()


				return 'stop'
			if not self.other_car:
				self.mutex.release()
				return 'idle'

			self.mutex.release()
			rospy.sleep(self.sleep_time)


	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()

if __name__ == "__main__":
	rospy.init_node('supervisor', anonymous=True)

	#rospy.wait_for_service("/Start_pure_pursuit")
	#rospy.wait_for_service("/Stop_pure_pursuit")

	sm = smach.StateMachine(outcomes=[])
	dispatcher=Dispatcher()

	with sm:
		smach.StateMachine.add('StateIdle', StateIdle(dispatcher),
		                       transitions={'run': 'StateRunning', 'stop': 'StateStopped', 'follow':'StateFollowing'})
		smach.StateMachine.add('StateRunning', StateRunning(dispatcher),
		                       transitions={'run':'StateRunning', 'stop':'StateStopped','follow':'StateFollowing'})
		smach.StateMachine.add('StateStopped', StateStopped(dispatcher),
		                       transitions={'stop':'StateStopped', 'run':'StateRunning', 'follow':'StateFollowing'})
		smach.StateMachine.add('StateFollowing', StateFollowing(dispatcher),
		                       transitions={'idle':'StateIdle', 'stop':'StateStopped','follow':'StateFollowing'})

	smach_thread = threading.Thread(target=sm.execute)
	smach_thread.start()

	rospy.spin()
	sm.request_preempt()

	smach_thread.join()

