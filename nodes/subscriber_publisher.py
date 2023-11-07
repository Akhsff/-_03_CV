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

# constants
ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")
ROS_IMAGE_TOPIC: Final[str] = "image"


class ImagePublisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/image_resized', Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/image_new", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            resized_image = cv2.resize(cv_image, (128, 128))
            resized_image_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")
            self.image_pub.publish(resized_image_msg)
            cv2.imshow("Original Image", cv_image)
            cv2.imshow("Resized Image", resized_image)

            cv2.waitKey(1)

            
        except Exception as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('image_publisher', anonymous=True)
    image_publisher = ImagePublisher()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")