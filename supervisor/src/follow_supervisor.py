#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32
#TODO
#import another car 


def idle_tn(danger=0, path_available = False, danger_threshold=0.5, another_car= False):
    if path_available and danger < danger_threshold and not another_car:
        return 'RUN', True, 'IDLE'
    elif danger < danger_threshold and another_car:
        return 'FOLLOW', True, 'IDLE'
    else:
        return 'IDLE', False, 'IDLE'

def run_tn(path_available = False, danger = 0, danger_threshold=0.5, another_car= False):
    if path_available and danger > danger_threshold:
        return 'STOP', True, 'RUN'
    elif another_car and danger < danger_threshold:
        return 'FOLLOW', True, 'RUN'
    else:
        return 'RUN', False, 'RUN'

def stop_tn(path_available = False, danger = 0, danger_threshold=0.5, another_car= False):
    if path_available and danger < danger_threshold and not another_car:
        return 'RUN', True, 'STOP'
    elif danger < danger_threshold and another_car:
        return 'FOLLOW', True, 'STOP'
    else:
        return 'STOP', False, 'STOP'

def follow_tn(path_available = False, danger = 0, danger_threshold=0.5, another_car= False):
    if path_available and not another_car:
        return 'RUN', True, 'FOLLOW'
    elif danger < danger_threshold and another_car:
        return 'FOLLOW', True, 'FOLLOW'
    else:
        return 'STOP', False, 'FOLLOW'

class StateMachine:
    def __init__(self, init_state, states_tn):
        self.transited = True
        self.state = init_state
        self.states_tn = states_tn
	self.path_available = False
	self.prev_state = init_state
	self.another_car= False
	# Access rosparams
	self.command_controller_pure_pursuit = rospy.get_param(rospy.get_name() + "/command_controller_topic")
	self.command_controller_follow = rospy.get_param(rospy.get_name() + "/command_controller_follow") #add in the launch file
	self.path_top = rospy.get_param(rospy.get_name() + "/path_topic")
	self.danger_top = rospy.get_param(rospy.get_name() + "/danger_topic")
        # Initialize subscriber
        
	self.sub_another_car=rospy.Subscriber("Bool_another_car", Bool, self.callback_another_car)
	self.sub_danger = rospy.Subscriber(self.danger_top, Float32, self.update_danger)
        self.sub_path = rospy.Subscriber(self.path_top, Path, self.update_path)
	self.pub_command_controller_pure_pursuit = rospy.Publisher(self.command_controller_pure_pursuit, Bool, queue_size=10) 
        self.pub_command_controller_follow = rospy.Publisher(self.command_controller_follow, Bool) #write in the PID controller

        self.danger = 1
 
    def callback_another_cat(self, bool_another_car):
	self.another_car=bool_another_car.data
   
    def update_danger(self, danger):
        self.danger = danger.data
        
    def update_path(self, path):
        self.path_available = True

    def advance_machine(self):
        args = {'danger': self.danger, 'path_available':self.path_available, 'another_car': self.another_car}
        self.state, self.transited, self.prev_state = self.states_tn[self.state](**args) #
        if self.transited:
            print('NEW STATE:', self.state)

    def run_service(self):
        if self.state == 'IDLE':
            pass
        if self.state == 'RUN':
            # request Start service Pure Pursuit
            if self.transited:
                msg = Bool()
                msg.data = True
                self.pub_command_controller_pure_pursuit.publish(msg)
            else:
                pass

        if self.state == 'STOP':
            # request Stop service Pure Pursuit
            if self.transited and self.prev_state=='FOLLOW':
                msg = Bool()
                msg.data = False
                self.pub_command_controller_pure_pursuit.publish(msg)
            else:
                pass
	    # request Stop service PID controller
	    if self.transited and self.prev_state=='RUN':
                msg = Bool()
                msg.data = False
                self.pub_command_controller_follow(msg)
            else:
                pass

	if self.state == 'FOLLOW':
            # request Stop service PID controller
            if self.transited:
                msg = Bool()
                msg.data = True
                self.pub_command_controller_follow.publish(msg)
            else:
                pass
            

if __name__ == "__main__":
    rospy.init_node('supervisor', anonymous=True)
    tns = {'IDLE': idle_tn, 'RUN': run_tn, 'STOP': stop_tn, 'FOLLOW': follow_tn}
    sm = StateMachine(init_state='IDLE', states_tn=tns)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Refresh the state
        sm.advance_machine()
        sm.run_service()
	rate.sleep()
