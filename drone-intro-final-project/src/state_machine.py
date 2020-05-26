from enum import Enum
from math import sqrt, cos, sin
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
import rospy
from simple_pid import PID

from message_tools import orientation_to_yaw, point_to_arr, arr_to_point

class StateMachine():
    class States(Enum):
        IDLE = "idle"
        WAITING_TO_ARRIVE = "wait_arrive"
        TAKE_OFF = "takeoff"

    def __init__(self):
        self._current_value = self.States.IDLE
        self._next_value = self.States.IDLE
        self.pose_error = Pose()
    
    def _pose_error_callback(self, topic = PoseStamped()):
        self.pose_error = topic.pose
    
    def set_params(self, params):
        self._mav1 = params
    
    def set_next_state(self, state):
        self._next_value = state
    
    def set_current_state(self, state):
        print("NEW STATE: " + str(state))
        self._current_value = state
    
    def execute(self):
        cur = self._current_value
        if cur == self.States.IDLE:
            pass

        elif cur == self.States.WAITING_TO_ARRIVE:
            if self._mav1.has_arrived():
                self.set_current_state(self._next_value)

        elif cur == self.States.TAKE_OFF:
            target = Point(0, 0, 10)
            self._mav1.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.IDLE)





