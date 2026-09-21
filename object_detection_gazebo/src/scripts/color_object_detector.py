#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorObjectDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('color_object_detector_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Subscriber to Gazebo raw camera topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        # Publisher for annotated output image
        self.image_pub = rospy.Publisher("/detected_object/image", Image, queue_size=1)
        
        rospy.loginfo("Color Object Detector Node initialized. Tracking Red Objects...")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # Convert BGR frame to HSV space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Defined HSV ranges for RED color (Red wraps around 0/180 degrees in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create combined mask for red color range
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Apply Morphological Operations to remove small camera noise
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        # Extract contours
        _, contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Minimum area threshold to ignore minor camera noise
            if area > 500:
                # Get bounding box coordinates
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate object centroid via image moments
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Draw visual overlays
                    cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, "Red Object", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    rospy.loginfo_throttle(2, "Red Object detected at Centroid Pixel Coordinates: (%d, %d)", cX, cY)

        try:
            # Publish modified image stream
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Output Error: %s", e)

if __name__ == '__main__':
    try:
        detector = ColorObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
