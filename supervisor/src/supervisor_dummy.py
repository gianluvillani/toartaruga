#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool

def idle_tn(danger=0, path_available = False, danger_threshold=0.5):
    if path_available:
        return 'RUN', True
    else:
        return 'IDLE', False

def run_tn(path_available = False, danger = 0, danger_threshold=0.5):
    if path_available and danger < danger_value:
        return 'RUN', False
    else:
        return 'STOP', True

def stop_tn(path_available = False, danger = 0, danger_threshold=0.5):
    if path_available and danger < danger_value:
        return 'RUN', True
    else:
        return 'STOP', False


# Service needed:
#    - Start pure_pursuit
#    - Stop  pure_pursuit


class StateMachine:
    def __init__(self, init_state, states_tn):
        self.transited = True
        self.state = init_state
        self.states_tn = states_tn
        # Initialize subscriber
        # Initialize services
        rospy.wait_for_service("/Start_pure_pursuit")
        rospy.wait_for_service("/Stop_pure_pursuit")
        self.sub_danger = rospy.Subscriber('/danger', Float32, self.update_danger)
        self.sub_path = rospy.Subscriber('/SVEA2/path', Path, self.update_path)
        self.pub_command_controller = rospy.Publisher('/controller_active', Bool)
        self.danger = 1
        
    def udpdate_danger(self, danger):
        self.danger = danger.data
        
    def udpdate_path(self, danger):
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
                self.pub_command_controller = rospy.publish(msg)
            else:
                pass

        if self.state == 'STOP':
            # request Stop service
            if self.transited:
                msg = Bool()
                msg.data = False
                self.pub_command_controller = rospy.publish(msg)
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

