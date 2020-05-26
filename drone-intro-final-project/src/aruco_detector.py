#!/usr/bin/env python
 
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ArUcoDetector():
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        subscribe_topic="/iris_fpv_cam/usb_cam/image_raw"
        publish_topic="/aruco_detector/marker_pose"
        self.marker_id=23
        self.camera_matrix=np.array([
            [277.191356, 0, 320.5],
            [0, 277.191356, 240.5],
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

        # self.pub = rospy.Publisher(
        #         publish_topic,
        #         markerpose,
        #         queue_size=1
        # )

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
                corners,
                self.aruco_length,
                self.camera_matrix,
                np.zeros(4)
            )
            xoffset = -2.5
            yoffset = -1.8
            print("TVECVS\n"+str(tvecs[0][0]-np.array([xoffset,yoffset,0])))


if __name__ == '__main__':
    try:
        node = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
