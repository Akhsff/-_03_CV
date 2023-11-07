#!/usr/bin/env python3
# encoding: utf-8

import rospy
import os
import cv2
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Final

# Constants
ROS_NODE_NAME: Final[str] = "camera_publisher_new"
ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")
ROS_IMAGE_TOPIC: Final[str] = "image_new"

def main() -> None:
    rospy.init_node(ROS_NODE_NAME)
    pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)
    publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)
    rospy.loginfo(f"Publishing camera image to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")
    rate = rospy.Rate(pub_frequency)
    
    bridge = CvBridge()
    
    # Open the camera (you may need to change the camera index if you have multiple cameras)
    cap = cv2.VideoCapture(0)  # 0 represents the default camera (change if necessary)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open the camera.")
        return
    
    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture frame from the camera.")
            break
        
        # Convert the OpenCV image to a sensor_msgs/Image message
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish the image message
        publisher.publish(image_msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()

