#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def publisher_node():
    cmd_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    xCur = Pose()
    xCur.position.x = 0
    xCur.position.y = 0

    xCur.orientation.x = 0
    #^^This is theta

    xEnd = Pose()
    xEnd.position.x = 2
    xEnd.position.y = 0.5
    xEnd.orientation.x = 3*math.pi/4


    xCorrect = False
    yCorrect = False
    thetaCorrect = False

    posTolerance = 0.01
    orTolerance = 2 * math.pi/180

    while not rospy.is_shutdown():

       #Check if at position. If so, STOP!!!
       if (xEnd.position.x - posTolerance <= xCur.position.x <= xEnd.position.x + posTolerance):
          xCorrect = True

       if (xEnd.position.y - posTolerance <= xCur.position.y <= xEnd.position.y + posTolerance):
          yCorrect = True

       if (xEnd.orientation.x - orTolerance <= xCur.orientation.x <= xEnd.orientation.x + orTolerance):
          thetaCorrect = True

       '''Algo: Check if position is correct. If not, rotate so bot faces
          goal position. Then move to goal position. Then check if
          orientation is correct. If not rotate. Then done.'''

       if (not xCorrect or not yCorrect or not thetaCorrect):
           #We want to rotate in desired direction and then move forward
           moveToPosition(xCur, xEnd, cmd_pub)
           xCur = xGoal

       rate.sleep()

    '''
    TODO: complete the publisher function here
    '''
    pass

def moveToPosition(xCurrent, xGoal, cmd_pub):
    distance = math.sqrt((xGoal.position.x-xCurrent.position.x)**2+(xGoal.position.y-xCurrent.position.y)**2)
    angleToTravel = math.atan((xGoal.position.y-xCurrent.position.y)/(xGoal.position.x-xCurrent.position.x))
    finalRotation = xGoal.orientation.x - xCurrent.orientation.x - angleToTravel
    print(distance)
    print(angleToTravel)
    motionCnt = 0
    rotationCnt = 0
    finalRotationCnt = 0
    twist = Twist()
    rate = rospy.Rate(1000)
    while True:
        if rotationCnt <= 10*1000*(angleToTravel):
            twist.angular.z=0.1
            rotationCnt+=1
        elif motionCnt <= 5000*distance:
            twist.angular.z = 0
            twist.linear.x=0.2
            motionCnt += 1
        elif finalRotationCnt <= 10*1000*(finalRotation):
            twist.linear.x=0
            twist.angular.z = 0.1
            finalRotationCnt += 1
        else:
            twist.angular.z = 0

        #rospy.loginfo(twist)
        cmd_pub.publish(twist)
        rate.sleep()


def main():

    try:
        rospy.init_node('lab02')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if _name_ == '_main_':
    main()