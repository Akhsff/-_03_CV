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

            # lower_red = np.array([355,200,100])
            # upper_red = np.array([360,255,255])
            # mask1 = cv.inRange(hsv, lower_red, upper_red)

            mask = mask0
  
            contours0, _ = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            # перебираем все найденные контуры в цикле
            for cnt in contours0:
                rect = cv.minAreaRect(cnt) # пытаемся вписать прямоугольник
                box = cv.boxPoints(rect) # поиск четырех вершин прямоугольника
                box = np.int0(box) # округление координат
                area = int(rect[1][0]*rect[1][1]) # вычисление площади
                if area > 3000:
                    cv.drawContours(cv_image,[box],0,(0,255,0),5)
            
          
            # img_gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
            # img_canny = cv.Canny(img_gray, 120, 300)
            # contours, _ = cv.findContours(img_canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            # curr_index = 1
            # curr_contour = contours[curr_index]

            # eps = 0.04 * cv.arcLength(curr_contour, closed=True)
            # approx_poly = cv.approxPolyDP(curr_contour, epsilon=eps, closed=True)

            # img_poly = cv_image.copy()
            # cv.drawContours(img_poly, [curr_contour], -1, (0,110,50), 2)
            # cv.drawContours(img_poly, [approx_poly], -1, (5,255,255), 3)

            # print(f'Area: {cv.contourArea(curr_contour)}, perimeter: {cv.arcLength(curr_contour, True)}')
            # print(f'Area: {cv.contourArea(approx_poly)}, perimeter: {cv.arcLength(approx_poly, True)}')

            # print(f'Number of vertices: {len(approx_poly)}')
            cv.imshow('contours', cv_image)
            cv.waitKey(1)

            # cv_image = self.bridge.imgmsg_to_cv2(data, "rgba8")
            # img = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
            # img_gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)


            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            # clahe_image = clahe.apply(img_gray)

            # blur = cv2.medianBlur(img_gray, 11)
            # img_canny = cv2.Canny(clahe_image, 50, 300)
            # img_sobel = cv2.Sobel(img_gray, -2, dx=1, dy=1, ksize=5, scale=3)

            # contours, _ = cv2.findContours(img_canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # img_countours = img.copy()
            # cv2.drawContours(img_countours, contours, -1, (0,255,0), 2)

            # cv2.imshow("ORIGINAL",img)
            # cv2.imshow("GRAY",blur)
            # cv2.imshow("CANNY", img_canny)
            # cv2.imshow("SOBEL", img_sobel)

            # cv2.imshow("countours", img_countours)
            
            # cv2.waitKey(1)
        except Exception as e:
                print(e)


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    image_converter = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")