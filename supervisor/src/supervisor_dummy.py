#!/usr/bin/env python
import rospy
danger_value = 0.5

def idle_tn(path_available = False):
    if path_available:
        return 'RUN'
    else:
        return 'IDLE'

def run_tn(path_available = False, danger = 0):
    if path_available and danger < danger_value:
        return 'RUN'
    else:
        return 'STOP'

def stop_tn(path_available = False, danger = 0):
    if path_available and danger < danger_value:
        return 'RUN'
    else:
        return 'STOP'


# Service needed:
#    - Start pure_pursuit
#    - Stop  pure_pursuit


class StateMachine:
    def __init__(self, init_state, states_tn):
        self.state = init_state
        self.states_tn = states_tn
        # Initialize subscriber
        # Initialize services
        rospy.wait_for_service("/Start_pure_pursuit")
        rospy.wait_for_service("/Stop_pure_pursuit")

    def advance_machine(self):
        args = {'danger': self.danger, 'path_available':self.path_available}
        self.state = self.states_tn[self.state](**args)

    def run_service(self):
        if self.state == 'IDLE':
            pass
        if self.state == 'RUN':
            # request Start service
            pure_pursuit_srv_start = rospy.ServiceProxy("/Start_pure_pursuit", Empty)
            pure_pursuit_srv()

        if self.state == 'STOP':
            # request Stop service
            pure_pursuit_srv_stop = rospy.ServiceProxy("/Stop_pure_pursuit", Empty)
            pure_pursuit_srv()

if __name__ == "__main__":
    rospy.init_node('supervisor', anonymous=True)
    tns = {'IDLE': idle_tn, 'RUN': run_tn, 'STOP': stop_tn}
    sm = StateMachine(init_state='IDLE', states_tn=tns)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Refresh the state
        sm.advance_machine()
        # Eventually sends signals to the other nodes in the form of services
        sm.run_service()
        rate.sleep()

