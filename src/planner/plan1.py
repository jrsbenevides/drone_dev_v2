#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from tf.transformations import euler_from_quaternion

def update_pose(poseDrone):
    
    global position_x, position_y, position_z

    position_x = poseDrone.pose.pose.position.x
    position_y = poseDrone.pose.pose.position.y
    position_z = poseDrone.pose.pose.position.z

def goToGlobal(x,y,z,yaw,t):

    x0 = x - posicao[0]
    y0 = y - posicao[1]
    z0 = z - posicao[2]
    psiD = yaw - rotacao

    r = [[np.cos(psiD), np.sin(psiD) * -1, 0], [np.sin(psiD), np.cos(psiD), 0], [0, 0, 1]]
    posDesired = np.dot(np.transpose(np.asarray(r)), np.asarray([x0, y0, z0]))

    positionDesired_x = posDesired[0]
    positionDesired_y = posDesired[1]
    positionDesired_z = posDesired[2]

    print('Altura Verificada')

    goToPoint(positionDesired_x,positionDesired_y,positionDesired_z,psiD,t)

def goToPoint(x,y,z,yaw,t):

    goalVector = PoseStamped()
    
    goalVector.pose.position.x = x
    goalVector.pose.position.y = y
    goalVector.pose.position.z = z
    goalVector.pose.orientation.x = yaw
    goalVector.pose.orientation.y = t
    goalVector.pose.orientation.z = 10      

    print('Indo...')
    targetMsg.publish(goalVector)

def stopPoint():

    goalVector = PoseStamped()
    
    goalVector.pose.orientation.z = 0      

    targetMsg.publish(goalVector)

def viconCallback(vicon):
    global r, psi0, po, rotacao, posicao, viconOne
    pv = vicon.transform.translation
    ov = vicon.transform.rotation
    roll, pitch, yaw = euler_from_quaternion([ov.x, ov.y, ov.z, ov.w])        
    if viconOne == 0:
        psi0 = yaw
        po = pv
        viconOne = 1
    else:
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        posicao = np.dot(np.transpose(np.asarray(r)), np.asarray([[pv.x - po.x], [pv.y - po.y], [pv.z - po.z]]))
        rotacao = yaw - psi0

def takeoff():
    print('Decolando...')
    myTakeOff.publish(Empty())

def land():
    print('Pousando')
    myLand.publish(Empty())

def initStateMachine():
    global startSM, viconOne, enableToken, token, takeOffBusy, timeTakeOff, stopBusy, timeStop, r, psi0, po
    print('Inicializando...')
    viconOne    = 0
    token       = 0
    enableToken = 1
    startSM     = 0
    takeOffBusy = 0
    timeTakeOff = -99
    timeStop    = -99
    stopBusy    = 0
    r = 0
    psi0 = 0
    po = 0
    
def checkToken(tokenLocal):
    global enableToken, token, takeOffBusy, timeTakeOff

    if(enableToken > 0):
        if(tokenLocal == 0):    
            if(takeOffBusy == 0):
                takeoff()
                takeOffBusy = 1
                timeTakeOff = rospy.get_rostime().secs
            else:
                if(rospy.get_rostime().secs - timeTakeOff < 5):
                    print('TakingOff')
                else:
                    token+=1
                    timeTakeOff = -99
        elif(tokenLocal == 1):
            goToPoint(0.0,0.0,0.6,0,6)
            enableToken = 2
        elif(tokenLocal == 2):
            goToPoint(3.0,1.00,0.0,0,12)
            enableToken = 2
        elif(tokenLocal == 3):
            print('Landing...')
            land()
            takeOffBusy = 0
            enableToken = 0

def readStatus(msg):
    global startSM, token, enableToken, timeTakeOff, stopBusy, timeStop
    
    intMsg = int(msg.data)

    # print('msg = ' + str(intMsg))

    if(startSM):
        if(intMsg == 0x00):
            if(enableToken == 1):
                checkToken(token)
            elif(enableToken == 3):
                if(rospy.get_rostime().secs - timeStop > 2):
                    token+=1        #trocar para pegar a ordenacao de maneira generica, sem sequencia
                    enableToken = 1
                    stopBusy = 0                
                    timeStop = -99
                    print('Parou...')
        elif(intMsg == 0xFF):
            if(enableToken == 2):
                print('Busy...')
                enableToken = 3
        elif(intMsg == 0xAA):
            if(enableToken == 3):
                if(stopBusy == 0):
                    stopPoint()
                    print('Parando...')
                    stopBusy = 1
                    timeStop = rospy.get_rostime().secs
        elif(intMsg == 0x77):
            print('MSG DO DIEGO') #lembrando que isso deve mudar o token, (e enableToken) dependendo de onde ele ta

if __name__ == '__main__':
    global position_x, position_y, position_z, startSM, viconOne, token, enableToken, takeOffBusy, timeTakeOff, stopBusy
    initStateMachine()
    myTakeOff   = rospy.Publisher("/bebop/takeoff", Empty, queue_size = 1)
    myLand      = rospy.Publisher("/bebop/land", Empty, queue_size = 1)
    targetMsg   = rospy.Publisher("/planejamento", PoseStamped, queue_size = 1)
    # rdStatus    = rospy.Subscriber("/statusPlanning", UInt8, readStatus)
    # poseEst     = rospy.Subscriber("/bebop/transf_position", Odometry, update_pose)
    vicon_sub   = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, viconCallback)
    rospy.init_node('plannerGlobal', anonymous=True)
    
    rospy.sleep(5.)
    startSM = 1

    try:
        takeoff()
        rospy.sleep(5.)
        goToGlobal(0,0,1.5,0,6)
        rospy.sleep(6.1)
        stopPoint()
        rospy.sleep(3.)
        goToGlobal(3.2361,0.4881,1.5,0,13)
        rospy.sleep(13.1)
        stopPoint()
        rospy.sleep(3.)
        land()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
