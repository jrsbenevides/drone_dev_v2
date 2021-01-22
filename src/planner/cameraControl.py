#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
	global angle
	myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
	rospy.init_node('plannerGlobal', anonymous=True)
	rospy.sleep(1.)
	angle = 0	
	lastTime = 0
	velRot = 3
	angMin = 80
	try:
		twist = Twist()
		while(angle > -1*angMin):
			time = rospy.get_rostime().secs +  1e-9*rospy.get_rostime().nsecs
			if(time - lastTime > 1.2):
				twist.angular.y = angle
				angle -= 16	
				# angle -= angMin/velRot	
				myCamera.publish(twist)
				lastTime = rospy.get_rostime().secs + 1e-9*rospy.get_rostime().nsecs
		print("FIM")
	except rospy.ROSInterruptException:
	    pass
