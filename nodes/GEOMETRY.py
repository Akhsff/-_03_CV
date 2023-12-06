#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2 as cv
import rospy
import rospkg
# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge
# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image
from typing import Final
import numpy as np
import math
import imutils
import random
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
            scale_factor = random.uniform(0.5, 2.0)
            angle = random.uniform(-30, 30)
            tx = random.randint(-50, 50)
            ty = random.randint(-50, 50)
            flip_horizontal = random.choice([True, False])


            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width = img.shape[:2]
            scaled_frame = cv.resize(img, (int(width * scale_factor), int(height * scale_factor)))

            rows, cols = img.shape[:2]
            translation_matrix = np.float32([[1, 0, tx], [0, 1, ty]])
            translated_frame = cv.warpAffine(scaled_frame, translation_matrix, (cols, rows))

            rotation_matrix = cv.getRotationMatrix2D((cols/2, rows/2), angle, 1)
            rotated_frame = cv.warpAffine(translated_frame, rotation_matrix, (cols, rows))
           
            if flip_horizontal:
                flipped_frame = cv.flip(rotated_frame, 1)
            else:
                flipped_frame = rotated_frame

            # cv.imshow("Scaled Frame", scaled_frame)
            # cv.imshow("Translated Frame", translated_frame)
            # cv.imshow("Rotated Frame", rotated_frame)
            cv.imshow("Результат", flipped_frame)
            cv.waitKey(1)
        except Exception as e:
                print(e)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    image_converter = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")