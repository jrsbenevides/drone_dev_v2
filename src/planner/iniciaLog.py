#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import UInt8

def callbackGlobalPlan(data):
    if data.data == 0xFF:
        os.system("rosrun log log_node")

if __name__ == "__main__":
    rospy.Subscriber("iniciaLog", UInt8, callbackGlobalPlan)
    rospy.init_node('Inicializador_Do_Log', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


