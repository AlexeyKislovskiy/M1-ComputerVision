#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

rospy.init_node('color_filter')
bridge = CvBridge()
color_ranges = {
    'red': [((0, 50, 70), (10, 255, 255)),
                ((159, 50, 70), (180, 255, 255))],
    'blue': [((90, 50, 70), (128, 255, 255))],
    'green': [((36, 50, 70), (89, 255, 255))]
}
color = rospy.get_param('~color', 'red')
ranges = color_ranges[color]

def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        current_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask = cv2.bitwise_or(mask, current_mask)

    filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    filtered_image_msg = bridge.cv2_to_imgmsg(filtered_image, "bgr8")
    filtered_pub.publish(filtered_image_msg)

rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
filtered_pub = rospy.Publisher('/usb_cam/filtered_image', Image, queue_size=10)
rospy.spin()