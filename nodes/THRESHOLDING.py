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
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            result = cv_image.copy()
            # # lower mask (0-10)
            lower_red = np.array((0, 144, 136), np.uint8)
            upper_red = np.array((10, 255, 255), np.uint8)
            mask = cv2.inRange(hsv, lower_red, upper_red)
            result = cv2.bitwise_and(result, result, mask=mask)

            cv2.imshow("mask",mask)
            cv2.imshow("result",result)
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