#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32

def idle_tn(danger=0, path_available = False, danger_threshold=0.5):
    if path_available and danger < danger_threshold:
        return 'RUN', True
    else:
        return 'IDLE', False

def run_tn(path_available = False, danger = 0, danger_threshold=0.5):
    if path_available and danger < danger_threshold:
        return 'RUN', False
    else:
        return 'STOP', True

def stop_tn(path_available = False, danger = 0, danger_threshold=0.5):
    if path_available and danger < danger_threshold:
        return 'RUN', True
    else:
        return 'STOP', False

class StateMachine:
    def __init__(self, init_state, states_tn):
        self.transited = True
        self.state = init_state
        self.states_tn = states_tn
	self.path_available = False
	# Access rosparams
	self.command_controller_top = rospy.get_param(rospy.get_name() + "/command_controller_topic")
	self.path_top = rospy.get_param(rospy.get_name() + "/path_topic")
	self.danger_top = rospy.get_param(rospy.get_name() + "/danger_topic")
        # Initialize subscriber
        
	self.sub_danger = rospy.Subscriber(self.danger_top, Float32, self.update_danger)
        self.sub_path = rospy.Subscriber(self.path_top, Path, self.update_path)
        self.pub_command_controller = rospy.Publisher(self.command_controller_top, Bool)

        self.danger = 1
        
    def update_danger(self, danger):
        self.danger = danger.data
        
    def update_path(self, path):
        self.path_available = True

    def advance_machine(self):
        args = {'danger': self.danger, 'path_available':self.path_available}
        self.state, self.transited = self.states_tn[self.state](**args)
        if self.transited:
            print('NEW STATE:', self.state)

    def run_service(self):
        if self.state == 'IDLE':
            pass
        if self.state == 'RUN':
            # request Start service
            if self.transited:
                msg = Bool()
                msg.data = True
                self.pub_command_controller.publish(msg)
            else:
                pass

        if self.state == 'STOP':
            # request Stop service
            if self.transited:
                msg = Bool()
                msg.data = False
                self.pub_command_controller.publish(msg)
            else:
                pass
            

if __name__ == "__main__":
    rospy.init_node('supervisor', anonymous=True)
    tns = {'IDLE': idle_tn, 'RUN': run_tn, 'STOP': stop_tn}
    sm = StateMachine(init_state='IDLE', states_tn=tns)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Refresh the state
        sm.advance_machine()
        sm.run_service()
        rate.sleep()

