#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import UInt8

def callbackGlobalPlan(data):
    if data.data == 0xFF:
        os.system("roslaunch orb_slam2_ros bebop2_orb_slam2.launch")

if __name__ == "__main__":
    rospy.Subscriber("iniciaOrbSlam", UInt8, callbackGlobalPlan)
    rospy.init_node('Inicializador_Do_Orbs_Slam', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


