#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String # For pub/sub
from sensor_msgs.msg import Image # Publisher
from geometry_msgs.msg import PoseArray # Publisher
from geometry_msgs.msg import Pose # Publisher
from geometry_msgs.msg import Point # Publisher
from geometry_msgs.msg import Quaternion # Publisher


from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from pouso_camera_inferior import landing_detector_main

# from detector import landing_detector_main

# # import os # Para teste
# # import time

# @@@@@@ Use this on target folder to make node executable: chmod +x [filename].py
def start_detector_video():

  video_stream = cv2.VideoCapture('teste_1_1310.avi')
  image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
  pub = rospy.Publisher('perception/target', PoseArray, queue_size=1)

  rospy.init_node('image_converter', anonymous=True)

  rostime_initial = rospy.get_rostime().secs

  bridge = CvBridge()
  while not rospy.is_shutdown():
    while video_stream.isOpened():
      ret, cv_image = video_stream.read()
      if ret == True:
        # filename = "image" + str(rospy.get_rostime().secs - rostime_initial) + ".jpg"
        # cv2.imwrite(filename, cv_image)

        cv_image,filtered,center,center_rotation = landing_detector_main(cv_image,camera_control_angle,bebop_z)

        if center:
          msg = build_PoseArray(center,center_rotation,140)

          pub.publish(msg)

        try:
          image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8")) # Publica a imagem como sensor_msg/Image
        except CvBridgeError as e:
          print(e)

def image_publisher():
    camera_control_angle = 80.0
    bebop_z = 2.5
    image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
    pub = rospy.Publisher('perception/target', PoseArray, queue_size=1)
    rospy.init_node('image_converter', anonymous=True)

    cap = cv2.VideoCapture("http://192.168.4.1:8000/stream.mjpg")

    bridge = CvBridge()

    while not rospy.is_shutdown():
      ret, cv_image = cap.read()
      if ret == True:
        cv_image,filtered,center,center_rotation = landing_detector_main(cv_image,camera_control_angle,bebop_z)

        if center:
          msg = build_PoseArray(center,center_rotation,140)

          pub.publish(msg)

        try:
          image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8")) # Publica a imagem como sensor_msg/Image
        except CvBridgeError as e:
          print(e)

def build_PoseArray(center,center_rotation,c):
  msg = PoseArray()
  for point in center:
    current_point = Point(float(242 - point[1]),float(302 - point[0]),point[2])
    current_quaternion  = Quaternion(center_rotation[0][0],center_rotation[0][1],center_rotation[0][2],c)
    current_pose = Pose(current_point,current_quaternion)
    msg.poses.append(current_pose)
  return(msg)

if __name__ == '__main__':
  try:
    # start_detector_video()
    image_publisher()
  except rospy.ROSInterruptException:
    pass
    
# rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/image_topic_2


#camera matrix
# 595.035494 0.000000 301.823568
# 0.000000 594.674498 242.978598
# 0.000000 0.000000 1.000000

# 'K = ', [595.0354940047112, 0.0, 301.82356774120103, 0.0, 594.6744984701004, 242.97859777091915, 0.0, 0.0, 1.0]