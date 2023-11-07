#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2
import rospy
import rospkg
# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge
# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image
from typing import Final
import numpy as np

# constants
ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")
ROS_IMAGE_TOPIC: Final[str] = "image"


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_new", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv_image)
            h_image = self.bridge.cv2_to_imgmsg(h, "mono8")
            s_image = self.bridge.cv2_to_imgmsg(s, "mono8")
            v_image = self.bridge.cv2_to_imgmsg(v, "mono8")
            h_pub = rospy.Publisher('/h_channel', Image, queue_size=1)
            s_pub = rospy.Publisher('/s_channel', Image, queue_size=1)
            v_pub = rospy.Publisher('/v_channel', Image, queue_size=1)
            h_pub.publish(h_image)
            s_pub.publish(s_image)
            v_pub.publish(v_image)
            cv2.imshow("h",h)
            cv2.imshow("s",s)
            cv2.imshow("v",v)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    image_converter = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")