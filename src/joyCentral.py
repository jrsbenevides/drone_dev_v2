#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist, Vector3, Pose
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class globalPlanner:
    def __init__(self):       
        self.nOfAgents = 5
        
        self.cmd_pub     = rospy.Publisher("/cmd_global", PoseArray, queue_size=1)
        
        self.cmd_sub     = rospy.Subscriber("/bebop/cmd_velMAS", Twist, self.callback_cmd)
        self.reset_sub    = rospy.Subscriber("/bebop/resetMAS", Empty, self.callback_reset) 
        self.land_sub    = rospy.Subscriber("/bebop/landMAS", Empty, self.callback_land)      
        self.takeoff_sub = rospy.Subscriber("/bebop/takeoffMAS", Empty, self.callback_takeoff)
        
        rospy.init_node('joy_central', anonymous=True)
        
        print("Starting Multiple Joy Node...")


    def callback_takeoff(self, trash):
        
        poseAr = PoseArray()
        cmd       = Pose()

        cmd.orientation.y   = 1.0

        for n in range(self.nOfAgents):
            poseAr.poses.append(cmd)

        self.cmd_pub.publish(poseAr)
    
    def callback_land(self, trash):

        poseAr = PoseArray()
        cmd       = Pose()     

        cmd.orientation.z   = 1.0

        for n in range(self.nOfAgents):
            poseAr.poses.append(cmd)

        self.cmd_pub.publish(poseAr)

    def callback_reset(self, trash):
        
        poseAr = PoseArray()
        cmd       = Pose()

        cmd.orientation.w   = 1.0

        for n in range(self.nOfAgents):
            poseAr.poses.append(cmd)

        self.cmd_pub.publish(poseAr)

    def callback_cmd(self, cmdvel):
       
        cmdvelglb = PoseArray()
        cmd       = Pose()

        cmd.position.x      = cmdvel.linear.x
        cmd.position.y      = cmdvel.linear.y
        cmd.position.z      = cmdvel.linear.z
        cmd.orientation.x   = cmdvel.angular.z
        cmd.orientation.y   = 0.0
        cmd.orientation.z   = 0.0
        cmd.orientation.w   = 0.0

        for n in range(self.nOfAgents):
            cmdvelglb.poses.append(cmd)

        #EXECUTA SEMPRE QUE CHEGA MENSAGEM NOVA NO TOPICO GLOBAL (VINDO DO DRONE_DEV)
        self.cmd_pub.publish(cmdvelglb)

def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()