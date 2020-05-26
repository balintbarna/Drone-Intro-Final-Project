#!/usr/bin/env python

# import base libs
import numpy as np
import sys
import signal
import math

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose

# import files in package
from mav import Mav
from state_machine import StateMachine
from message_tools import *

class MainNode():
    def __init__(self):
        rospy.init_node('default_offboard', anonymous=True)
        self.rate = rospy.Rate(20)
        self.mav1 = Mav("mavros")

        self._command_sub = rospy.Subscriber("pose_command", String, self._pose_command_callback)

        # wait for FCU connection
        self.mav1.wait_for_connection()
        print("mavs connected")

        self.sm = StateMachine()
        self.sm.set_params(self.mav1)
        self.sm.set_current_state(self.sm.States.TAKE_OFF)
    
    def loop(self):
        # enter the main loop
        while (True):
            # print "Entered whiled loop"
            self.sm.execute()
            self.rate.sleep()
        
    def _pose_command_callback(self, topic = String()):
        text = topic.data
        textarr = np.array(text.split(";"))
        arr = textarr.astype(np.float)
        print("new target pose: "+str(arr.tolist()))
        pose = create_setpoint_message_xyz_yaw(arr[0], arr[1], arr[2], arr[3])
        self.mav1.set_target_pose(pose)
        self.sm.set_next_state(self.sm.States.IDLE)
        self.sm.set_current_state(self.sm.States.WAITING_TO_ARRIVE)
        pass


def main():
    node = MainNode()
    node.loop()

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()