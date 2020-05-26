#!/usr/bin/env python
 
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose

class ArUcoDetector():
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        subscribe_topic="/iris_fpv_cam/usb_cam/image_raw"
        publish_topic="/aruco_detector/marker_pose"
        self.marker_id=23
        self.camera_matrix=np.array([
            [277.191356, 0, 160],
            [0, 277.191356, 120],
            [0, 0, 1]
        ])
        self.aruco_length=2.0
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            subscribe_topic,
            Image,
            callback=self.on_new_image,
            queue_size=1
        )

        self.pub = rospy.Publisher(
                publish_topic,
                Point,
                queue_size=1
        )

    def on_new_image(self, topic):
        frame = self.bridge.imgmsg_to_cv2(topic, "bgr8")
        corners, ids, rejected_pts = cv2.aruco.detectMarkers(
            frame, 
            self.dictionary, 
            parameters=self.parameters
        )
        if ids == None:
            return
        if self.marker_id in ids:
            # TODO filter for ID

            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                corners[0],
                self.aruco_length,
                self.camera_matrix,
                np.zeros(4)
            )
            tvec = tvecs[0][0]
            print(tvec)
            self.pub.publish(Point(tvec[0], tvec[1], tvec[2]))


if __name__ == '__main__':
    try:
        node = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
