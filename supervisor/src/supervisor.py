#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Path
import threading
from control_hierarchy.dispatcher import Dispatcher
import time 


class StateIdle(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['stop', 'run', 'follow'])
		self.mutex = threading.Lock()
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_emergency_stop = rospy.Subscriber('/emergency_stop', Float32, self.emergency_stop_callback)
		self.sub_path = rospy.Subscriber('/race_course', Path, self.path_callback)
		self.sub_follow = rospy.Subscriber('/other_car', Bool, self.other_car_callback)
		self.sub_memorization = rospy.Subscriber('/start_memorization', Bool, self.memorize_path_callback)
		self.sub_start_publish = rospy.Subscriber('/start_publish_top', Float32, self.start_publish_callback)
		self.pub_start_stop_controller = rospy.Publisher('/start_stop_controller', Bool)
		self.emergency_stop = 0.0
		self.threshold = 0.5
		self.start_memorization = False
		self.sleep_time = 0.01
		self.path = None
		self.other_car = False
		self.start_publish_top = 0.0

	def execute(self, ud):
		rospy.logerr("IDLE")
		self.dispatcher = dispatcher
		self.dispatcher.switch_control('stop')
		self.dispatcher.publish_control()
		while not rospy.is_shutdown():
			self.mutex.acquire()
			if self.other_car:
				if self.emergency_stop > self.threshold:
					self.mutex.release()
					return 'stop'
				else:
					self.mutex.release()
					return 'follow'

			if self.path != None:
				if self.emergency_stop > self.threshold:
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

	def emergency_stop_callback(self, msg):
		self.mutex.acquire()
		self.emergency_stop = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()

	def memorize_path_callback(self, msg):
		self.mutex.acquire()
		self.start_memorization = msg.data
		self.mutex.release()

	def start_publish_callback(self, msg):
		self.mutex.acquire()
		self.start_publish_top = msg.data
		self.mutex.release()

class StateRunning(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['run', 'stop', 'follow'])
		self.mutex = threading.Lock()
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_emergency_stop = rospy.Subscriber('/emergency_stop', Float32, self.emergency_stop_callback)
		self.sub_follow = rospy.Subscriber('/other_car', Bool, self.other_car_callback)
		self.sub_memorization = rospy.Subscriber('/start_memorization', Bool, self.memorize_path_callback)
		self.sub_start_publish = rospy.Subscriber('/start_publish_top', Float32, self.start_publish_callback)		
		self.danger = 0.0
		self.emergency_stop = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.01
		self.other_car = False
		self.start_memorization = False
		self.start_publish_top = 1.0

	def execute(self, ud):
		rospy.logerr("RUN")
		#t0 = time.time()
		self.dispatcher = dispatcher
		self.dispatcher.switch_control('pure_pursuit')
		while not rospy.is_shutdown():
			self.mutex.acquire()
			#rospy.logerr("in run before publisher the control signals")
			self.dispatcher.publish_control()
			#t1 = time.time()
			#rospy.logerr("The time in RUN : %s",t1-t0)
			if self.emergency_stop > self.threshold:
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

	def emergency_stop_callback(self, msg):
		self.mutex.acquire()
		self.emergency_stop = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()

	def memorize_path_callback(self, msg):
		self.mutex.acquire()
		self.start_memorization = msg.data
		self.mutex.release()

	def start_publish_callback(self, msg):
		self.mutex.acquire()
		self.start_publish_top = msg.data
		self.mutex.release()


class StateStopped(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['idle', 'stop', 'run', 'follow'])
		self.mutex = threading.Lock()
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_emergency_stop = rospy.Subscriber('/emergency_stop', Float32, self.emergency_stop_callback)
		self.sub_other_car = rospy.Subscriber('/other_car', Bool, self.other_car_callback)
		self.sub_memorization = rospy.Subscriber('/start_memorization', Bool, self.memorize_path_callback)
		self.sub_start_publish = rospy.Subscriber('/start_publish_top', Float32, self.start_publish_callback)	
		self.danger = 0.0
		self.emergency_stop = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.01
		self.other_car = False
		self.start_memorization = False
		self.start_publish_top = 0.0
		# service call: stop tracking

	def execute(self, ud):
		rospy.logerr("STOP")
		self.dispatcher = dispatcher
		self.dispatcher.switch_control('stop')
		self.dispatcher.publish_control()
		while not rospy.is_shutdown():
			self.mutex.acquire()
			if self.emergency_stop < self.threshold:
				self.mutex.release()
				return 'idle'

			self.mutex.release()
			rospy.sleep(self.sleep_time)

	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def emergency_stop_callback(self, msg):
		self.mutex.acquire()
		self.emergency_stop = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()

	def memorize_path_callback(self, msg):
		self.mutex.acquire()
		self.start_memorization = msg.data
		self.mutex.release()

	def start_publish_callback(self, msg):
		self.mutex.acquire()
		self.start_publish_top = msg.data
		self.mutex.release()

class StateFollowing(smach.State):

	def __init__(self, dispatcher):
		smach.State.__init__(self, outcomes=['stop', 'idle', 'follow'])
		self.mutex = threading.Lock()
		self.sub_danger = rospy.Subscriber('/danger', Float32, self.danger_callback)
		self.sub_emergency_stop = rospy.Subscriber('/emergency_stop', Float32, self.emergency_stop_callback)
		self.sub_other_car = rospy.Subscriber('/other_car', Bool, self.other_car_callback)
		self.sub_memorization = rospy.Subscriber('/start_memorization', Bool, self.memorize_path_callback)
		self.sub_start_publish = rospy.Subscriber('/start_publish_top', Float32, self.start_publish_callback)		
		self.danger = 0.0
		self.emergency_stop = 0.0
		self.threshold = 0.5
		self.sleep_time = 0.01
		self.start_memorization= False
		self.start_publish_top = 0.0
		self.other_car= True

	def execute(self, ud):
		rospy.logerr("FOLLOW")
		self.dispatcher = dispatcher
		self.dispatcher.switch_control('pid')
		while not rospy.is_shutdown():
			self.mutex.acquire()
			self.dispatcher.publish_control()
			if not self.other_car:
				self.mutex.release()
				return 'idle'
			if self.emergency_stop > 2*self.threshold:
				self.mutex.release()
				return 'stop'


			self.mutex.release()
			rospy.sleep(self.sleep_time)


	def danger_callback(self, msg):
		self.mutex.acquire()
		self.danger = msg.data
		self.mutex.release()

	def emergency_stop_callback(self, msg):
		self.mutex.acquire()
		self.emergency_stop = msg.data
		self.mutex.release()

	def other_car_callback(self, msg):
		self.mutex.acquire()
		self.other_car = msg.data
		self.mutex.release()

	def memorize_path_callback(self, msg):
		self.mutex.acquire()
		self.start_memorization = msg.data
		self.mutex.release()

	def start_publish_callback(self, msg):
		self.mutex.acquire()
		self.start_publish_top = msg.data
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
		                       transitions={'idle':'StateIdle', 'stop':'StateStopped', 'run':'StateRunning', 'follow':'StateFollowing'})
		smach.StateMachine.add('StateFollowing', StateFollowing(dispatcher),
		                       transitions={'idle':'StateIdle', 'stop':'StateStopped','follow':'StateFollowing'})

	smach_thread = threading.Thread(target=sm.execute)
	smach_thread.start()

	rospy.spin()
	sm.request_preempt()

	smach_thread.join()

