#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ObjectTFBroadcaster:
    def __init__(self):
        rospy.init_node('object_tf_broadcaster_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Camera Intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        # Subscribers
        self.info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # Publisher
        self.image_pub = rospy.Publisher("/detected_object/image_tf", Image, queue_size=1)
        
        rospy.loginfo("TF Broadcaster Node initialized. Subscribed to camera feed...")

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.cx = msg.K[2]
        self.fy = msg.K[4]
        self.cy = msg.K[5]
        self.camera_info_received = True

    def image_callback(self, data):
        if not self.camera_info_received:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red object segmentation
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        _, contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 500:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])

                    # Estimated depth along camera optical Z-axis
                    Z_c = 1.25

                    # Back-project 2D pixel to 3D Camera coordinates
                    X_c = (u - self.cx) * Z_c / self.fx
                    Y_c = (v - self.cy) * Z_c / self.fy

                    # Broadcast TF transform for target object relative to camera_link
                    # Note: Using camera frame orientation conventions
                    self.tf_broadcaster.sendTransform(
                        (X_c, Y_c, Z_c),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "detected_red_ball",
                        "camera_link"
                    )

                    # Annotate frame
                    cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, "TF Frame: detected_red_ball", (u - 60, v - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    rospy.loginfo_throttle(2, "Broadcasting TF frame 'detected_red_ball' at (%.2f, %.2f, %.2f)", X_c, Y_c, Z_c)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Output Error: %s", e)

if __name__ == '__main__':
    try:
        broadcaster = ObjectTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass