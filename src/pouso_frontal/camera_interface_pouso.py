#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String # For pub/sub
from sensor_msgs.msg import CompressedImage # Subscriber
from sensor_msgs.msg import Image # Publisher
from geometry_msgs.msg import PoseArray # Publisher
from geometry_msgs.msg import Pose # Publisher
from geometry_msgs.msg import Point # Publisher
from geometry_msgs.msg import Quaternion # Publisher

from geometry_msgs.msg import Twist # Camera Control
from geometry_msgs.msg import TransformStamped # Bebob Z

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from detector import landing_detector_main

# @@@@@@ Use this on target folder to make node executable: chmod +x [filename].py

class image_converter:

  def __init__(self):
    self.camera_control_angle = 0.0
    self.bebop_z = 0.0

    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
    self.pub = rospy.Publisher('perception/target', PoseArray, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/bebop/image_raw/compressed",CompressedImage,self.callback, queue_size =1, buff_size=24*480*856)

    self.image_sub = rospy.Subscriber("/bebop/camera_control",Twist,self.callback_camera_control, queue_size =1) # Camera Control

    self.image_sub = rospy.Subscriber("/vicon/bebop/bebop",TransformStamped,self.callback_bebop_z, queue_size =1) # Bebob Z

  def callback(self,data):
    """
    Converte a imagem do bebop para uma imagem no OpenCV

    :param data: sensor_msg/CompressedImage. Imagem obtido no topico.
    :return: cv_image. cv::Mat. Imagem no ambiente do OpenCV
    
    """
    try:
      np_arr = np.fromstring(data.data, np.uint8) # sensor_msgs/CompressedImage
      cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # sensor_msgs/CompressedImage
      # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # sensor_msgs/Image
    except CvBridgeError as e:
      print(e)

    # # @@@@@@ Start @@@@@@
    
    # filename = "image" + str(rospy.get_rostime().secs - rostime_initial) + ".jpg"
    # cv2.imwrite(filename, cv_image)
    center = ()
    center_rotation = ()
    cv_image,filtered,center,center_rotation = landing_detector_main(cv_image,self.camera_control_angle,self.bebop_z)

    if center:
      msg = self.build_PoseArray(center,center_rotation)
      self.pub.publish(msg)

    # # @@@@@@ End @@@@@@

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3) 

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8")) # Publica a imagem como sensor_msg/Image
    except CvBridgeError as e:
      print(e)

  def build_PoseArray(self,center,center_rotation):
    msg = PoseArray()
    for point in center:
      current_point = Point(float(302 - point[0]),float(242 - point[1]),point[2])
      current_quaternion  = Quaternion(center_rotation[0][0],center_rotation[0][1],center_rotation[0][2],0.0)
      current_pose = Pose(current_point,current_quaternion)
      msg.poses.append(current_pose)
    return(msg)

  def callback_camera_control(self,data):
    self.camera_control_angle = data.angular.y

  def callback_bebop_z(self,data):
    self.bebop_z = data.transform.translation.z

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  global rostime_initial
  rostime_initial = rospy.get_rostime().secs

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
