from enum import Enum
from math import sqrt, cos, sin
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
import rospy

from message_tools import orientation_to_quat_array, orientation_to_yaw, yaw_to_orientation, point_to_arr, arr_to_point, create_setpoint_message_pos_yaw, create_setpoint_message_xyz_yaw
from scipy.spatial.transform import Rotation 

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
            # target = create_setpoint_message_xyz_yaw(1, 1, 10, 0)
            target = create_setpoint_message_xyz_yaw(7, 7, 10, 0)
            self._mav1.max_speed = 3
            self._mav1.set_target_pose(target)
            self.set_current_state(self.States.WAITING_TO_ARRIVE)
            self.set_next_state(self.States.CLOSE_POSE_ERROR)
            self._mav1.takeoff()
        
        elif cur == self.States.CLOSE_POSE_ERROR:
            # check if altitude is very low
            position = self._mav1.current_pose.pose.position
            current_pos = point_to_arr(position)
            altitude = current_pos[2]
            if altitude < 0.5:
                # land
                self._mav1.set_target_pos(position)
                self.set_current_state(self.States.IDLE)
                self._mav1.land()
                return
            if self.pose_error == None:
                return
            pose_error_arr = point_to_arr(self.pose_error)
            self.pose_error = None
            magnitude = np.linalg.norm(pose_error_arr)
            # check if limit is reached
            if magnitude < self.max_pose_error:
                self.set_current_state(self.States.IDLE)
                print("ALIGNED")
                return
            # match coordinate system to world
            (rx, ry, rz) = (-90, 180, 0)
            rm = Rotation.from_euler('zyx', [rx, ry, rz], degrees=True)
            pose_error_arr = rm.apply(pose_error_arr)
            # correct coordinate system for rotation of drone
            orientation = self._mav1.current_pose.pose.orientation
            quat_arr = orientation_to_quat_array(orientation)
            rm = Rotation.from_quat(quat_arr)
            pose_error_arr = rm.apply(pose_error_arr)
            # if XY error is large, zero error in Z, so it has change to center before it goes out of picture
            xyarr = pose_error_arr[:2]
            xymagn = np.linalg.norm(xyarr)
            if xymagn > 0.5:
                pose_error_arr[2] = 0
            print(pose_error_arr)
            # calc target pos
            target_pos = current_pos + pose_error_arr
            target_point = arr_to_point(target_pos)
            # send
            self._mav1.max_speed = 1
            self._mav1.set_target_pos(target_point)
