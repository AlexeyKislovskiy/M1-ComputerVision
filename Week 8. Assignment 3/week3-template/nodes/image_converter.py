#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        out_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(out_msg)
    except Exception as e:
        rospy.logerr("Error")

rospy.init_node('image_converter')
pub = rospy.Publisher('/image_color', Image, queue_size=10)
sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback)
rospy.spin()