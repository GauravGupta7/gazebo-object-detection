#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class Object3DPositionEstimator:
    def __init__(self):
        rospy.init_node('object_3d_position_estimator_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Intrinsic parameters (will be dynamically updated via CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        # Subscriber for Camera Info (Intrinsics)
        self.info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)
        
        # Subscriber for Camera RGB Image
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # Publisher for visual tracking output
        self.image_pub = rospy.Publisher("/detected_object/image_3d", Image, queue_size=1)
        
        rospy.loginfo("3D Position Estimator Node initialized. Waiting for /camera/camera_info...")

    def camera_info_callback(self, msg):
        # Extract intrinsic values from K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.K[0]
        self.cx = msg.K[2]
        self.fy = msg.K[4]
        self.cy = msg.K[5]
        self.camera_info_received = True

    def image_callback(self, data):
        if not self.camera_info_received:
            rospy.logwarn_throttle(5, "Waiting for camera intrinsic parameters...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Dual HSV range for Red color segmentation
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
                    u = int(M["m10"] / M["m00"])  # Pixel coordinate u (X in image)
                    v = int(M["m01"] / M["m00"])  # Pixel coordinate v (Y in image)

                    # Estimated distance along optical Z-axis (metres) based on fixed camera setup
                    # (In RGB-D systems, Z_c is sampled directly from the depth image buffer)
                    Z_c = 1.25  

                    # Back-projection equations from 2D pixel to 3D camera coordinates
                    X_c = (u - self.cx) * Z_c / self.fx
                    Y_c = (v - self.cy) * Z_c / self.fy

                    # Draw visual annotations
                    x_box, y_box, w_box, h_box = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x_box, y_box), (x_box + w_box, y_box + h_box), (0, 255, 0), 2)
                    cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)

                    text = "3D Pos (m): X={:.2f}, Y={:.2f}, Z={:.2f}".format(X_c, Y_c, Z_c)
                    cv2.putText(cv_image, text, (x_box, y_box - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    rospy.loginfo_throttle(2, "Calculated 3D Position in Camera Frame -> X: %.2f m, Y: %.2f m, Z: %.2f m", X_c, Y_c, Z_c)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Output Error: %s", e)

if __name__ == '__main__':
    try:
        estimator = Object3DPositionEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass