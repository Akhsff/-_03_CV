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
            hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV ) # меняем цветовую модель с BGR на HSV
            lower_red = np.array([0,144,136])
            upper_red = np.array([10,255,255])
            mask0 = cv.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([355,200,100])
            upper_red = np.array([360,255,255])
            mask1 = cv.inRange(hsv, lower_red, upper_red)
            mask = mask0+mask1
            
            contours, _ = cv.findContours(mask.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                rect = cv.minAreaRect(cnt) # пытаемся вписать прямоугольник
                box = cv.boxPoints(rect) # поиск четырех вершин прямоугольника
                box = np.int0(box) # округление координат
                area = int(rect[1][0]*rect[1][1]) # вычисление площади
                if area > 3000:
                    cv.drawContours(cv_image,[box],0,(0,255,0),2)
            
            for contour in contours:
                epsilon = 0.01 * cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 3:
                    print("Треугольник")
                elif len(approx) == 4:
                    x, y, w, h = cv.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if aspect_ratio >= 0.95 and aspect_ratio <= 1.05:
                        print("Квадрат")
                    else:
                        print("Прямоугольник")
                else:
                    print("Другая фигура")
                
                cv.drawContours(cv_image, [approx], 0, (255,0, 0), 2)
                
            cv.imshow('contours', cv_image)
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