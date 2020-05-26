from enum import Enum
from math import sqrt, cos, sin
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
import rospy
from simple_pid import PID

from message_tools import orientation_to_yaw, yaw_to_orientation, point_to_arr, arr_to_point, create_setpoint_message_pos_yaw

class StateMachine():
    class States(Enum):
        IDLE = "idle"
        WAITING_TO_ARRIVE = "wait_arrive"
        TAKE_OFF = "takeoff"
        CLOSE_POSE_ERROR = "pose_error"

    def __init__(self):
        self._current_value = self.States.IDLE
        self._next_value = self.States.IDLE
        self.pose_error = Point()

        sub = rospy.Subscriber("/aruco_detector/marker_pose", Point, self._pose_error_callback)

        self.max_angle_error = 0.1
        self.max_pose_error = 0.1
    
    def _pose_error_callback(self, topic = PoseStamped()):
        self.pose_error = topic
    
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
            target = Point(3, 3, 10)
            self._mav1.set_target_pos(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.CLOSE_POSE_ERROR)
        
        elif cur == self.States.CLOSE_POSE_ERROR:
            if self.pose_error == None:
                return
            pose_error_arr = point_to_arr(self.pose_error)
            # angle_error = orientation_to_yaw(self.pose_error.orientation)
            self.pose_error = None
            magnitude = np.linalg.norm(pose_error_arr)
            # check if limit is reached
            if magnitude < self.max_pose_error:
                self.set_current_state(self.States.IDLE)
                print("ALIGNED")
                return
            # calc angle
            current_yaw = orientation_to_yaw(self._mav1.current_pose.pose.orientation)
            # target_yaw = current_yaw + angle_error
            # calc pos
            current_pos = point_to_arr(self._mav1.current_pose.pose.position)
            target_pos = current_pos - pose_error_arr
            target_point = arr_to_point(target_pos)
            # send
            # target_pose = create_setpoint_message_pos_yaw(target_point, target_yaw)
            self._mav1.set_target_pos(target_point)
