#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist, Vector3
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class globalPlanner:
    def __init__(self):
        self.agent = 0    #Defines ID number of this agent (replace for hardware definition)
        self.flagBusy = False
        self.timeout = 10
        
        self.cmd_sub    = rospy.Subscriber("/cmd_global", PoseArray, self.callback_cmd)
        self.odom_sub   = rospy.Subscriber("/bebop/odom", Odometry, self.callback_imu)      #APENAS DEBUG

        self.takeoff    = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land       = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.reset      = rospy.Publisher("/bebop/reset", Empty, queue_size=10)
        self.cmd_pub    = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        self.odom_pub   = rospy.Publisher("/odom_global", Odometry, queue_size=1)        
        rospy.init_node('ncs_slave', anonymous=True)
        self.lastTime = rospy.get_rostime().secs
        
        print("comecou")

    def callback_imu(self, odom_imu):
        odomimu = Odometry()

        # odomimu.pose.pose.position.x = odom_imu.pose.pose.position.x
        # odomimu.pose.pose.position.y = odom_imu.pose.pose.position.y
        # odomimu.pose.pose.position.z = odom_imu.pose.pose.position.z
        odomimu = odom_imu

        #TENTAR SUBSTITUIR POR UMA UNICA CHAMADA!
        odomimu.header.stamp.secs = rospy.get_rostime().secs
        odomimu.header.stamp.nsecs = rospy.get_rostime().nsecs

        odomimu.header.frame_id = "0"
        #EXECUTA TODA VEZ QUE CHEGA UPDATE NA IMU
        self.odom_pub.publish(odomimu)

    def callback_cmd(self, cmdvelglb):
        
        cmdvel = Twist()
        
        if(self.flagBusy == True):                                  # Does the timeout handling for taking off and landing
            if(rospy.get_rostime().secs - self.lastTime >= self.timeout):
                self.flagBusy = False
                self.lastTime = rospy.get_rostime()

        if(cmdvelglb.poses[self.agent].orientation.w > 0):    # First priority goes to reset (emergency)
            self.reset.publish(Empty())
        elif(cmdvelglb.poses[self.agent].orientation.z > 0):  # Then to landing
            if(self.flagBusy == False):
                print("###Landing")            
                self.land.publish(Empty())
                print("###Done")  
                self.flagBusy = True
                self.lastTime = rospy.get_rostime().secs     
        elif(cmdvelglb.poses[self.agent].orientation.y > 0): # Then to taking off
            if(self.flagBusy == False):
                print("###Taking Off")            
                self.takeoff.publish(Empty())
                print("###Done")    
                self.flagBusy = True
                self.lastTime = rospy.get_rostime().secs
        else:                                               # Then to regular flight
            cmdvel.linear.x = cmdvelglb.poses[self.agent].position.x
            cmdvel.linear.y = cmdvelglb.poses[self.agent].position.y
            cmdvel.linear.z = cmdvelglb.poses[self.agent].position.z
            cmdvel.angular.x = 0
            cmdvel.angular.y = 0
            cmdvel.angular.z = cmdvelglb.poses[self.agent].orientation.x

            #EXECUTA SEMPRE QUE CHEGA MENSAGEM NOVA NO TOPICO GLOBAL (VINDO DO DRONE_DEV)
            self.cmd_pub.publish(cmdvel)

def main():
    
    globalPlanner()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()