#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from template.msg import MotionDetection

bridge = CvBridge()
prev_frame = None
motion_threshold = 1000

def motion_detection_callback(msg):
    global prev_frame
    current_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    if prev_frame is None:
        prev_frame = current_frame
        return
    frame_diff = cv2.absdiff(prev_frame, current_frame)
    gray_diff = cv2.cvtColor(frame_diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_diff, 25, 255, cv2.THRESH_BINARY)
    diff_pixels = np.sum(thresh == 255)
    detected = diff_pixels > motion_threshold
    motion_msg = MotionDetection()
    motion_msg.header.stamp = rospy.Time.now()
    motion_msg.header.frame_id = "camera_frame"
    motion_msg.detected = detected
    motion_msg.diff = diff_pixels
    motion_pub.publish(motion_msg)
    prev_frame = current_frame

rospy.init_node('motion_detector', anonymous=True)
rospy.Subscriber('/usb_cam/image_raw', Image, motion_detection_callback)
motion_pub = rospy.Publisher('/usb_cam/motion_detection', MotionDetection, queue_size=10)
rospy.spin()