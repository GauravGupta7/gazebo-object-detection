#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetectorPython:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_detector_python_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera feed from Gazebo
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # Publisher for annotated output image
        self.image_pub = rospy.Publisher("/detected_object/image", Image, queue_size=1)
        
        rospy.loginfo("Python Object Detector Node Started. Subscribed to /camera/rgb/image_raw")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV BGR image (NumPy array)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Convert BGR frame to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Broad HSV range to segment neutral/grey objects in Gazebo shapes.world
        lower_bound = np.array([0, 0, 50])
        upper_bound = np.array([180, 50, 200])

        # Create binary mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Extract contours from the mask
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter noise by min contour area
            if area > 1000:
                # Draw green bounding box
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate centroid using image moments
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Draw red centroid dot and text overlay
                    cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, "Object Centroid", (cX - 40, cY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        try:
            # Convert modified OpenCV image back to ROS Image message and publish
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Output Error: %s", e)

if __name__ == '__main__':
    try:
        detector = ObjectDetectorPython()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass