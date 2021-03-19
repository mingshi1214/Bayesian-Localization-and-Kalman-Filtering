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
    
    #Initializing array of positions to move to
    xA = Pose()
    xB = Pose()
    xC = Pose()
    xA.position.x = 1
    xA.position.y = 0
    xA.orientation.x = 2*math.pi/4
    xB.position.x = 1
    xB.position.y = 1
    xB.orientation.x = math.pi
    xC.position.x = 0
    xC.position.y = 1
    xC.orientation.x = 3*math.pi/2
    poses = [xA, xB, xC, xCur]
    posNumber = 0

    while not rospy.is_shutdown():
       '''Algo: Check if position is correct. If not, rotate so bot faces
          goal position. Then move to goal position. Then check if
          orientation is correct. If not rotate. Then done.'''
        #We want to rotate in desired direction and then move forward
       if (posNumber < len(poses)):
          moveToPosition(xCur, poses[posNumber], cmd_pub)
          xCur = poses[posNumber]
          posNumber += 1
          print("posNumber after incr: " + str(posNumber))

       rate.sleep()
       
    '''
    TODO: complete the publisher function here
    '''
    pass

def moveToPosition(xCurrent, xGoal, cmd_pub):
    distance = math.sqrt((xGoal.position.x-xCurrent.position.x)**2+(xGoal.position.y-xCurrent.position.y)**2)
    angleToTravel = 0
    if (xGoal.position.x-xCurrent.position.x == 0):
       if (xGoal.position.y-xCurrent.position.y > 0):
          angleToTravel = math.pi/2
       else:
          angleToTravel = 3*(math.pi/2)
    else:
       angleToTravel = math.atan((xGoal.position.y-xCurrent.position.y)/(xGoal.position.x-xCurrent.position.x))
    angleToTravel = (angleToTravel - xCurrent.orientation.x)%(math.pi)
    print("Current angle: " + str(xCurrent.orientation.x))
    finalRotation = (xGoal.orientation.x - xCurrent.orientation.x - angleToTravel)%(2*math.pi)
    print("Distance: " + str(distance))
    print("angleToTravel: " + str(angleToTravel))
    print("finalRotation: " + str(finalRotation))
    motionCnt = 0
    rotationCnt = 0
    finalRotationCnt = 0
    twist = Twist()
    rate = rospy.Rate(500)
    while True:
        if rotationCnt <= 2.5*500*(angleToTravel):
            twist.angular.z=0.4
            rotationCnt+=1
        elif motionCnt <= 2500*distance:
            twist.angular.z = 0
            twist.linear.x=0.2
            motionCnt += 1
        elif finalRotationCnt <= 2.5*500*(finalRotation):
            twist.linear.x=0
            twist.angular.z = 0.4
            finalRotationCnt += 1
        else:
            twist.angular.z = 0
            twist.linear.x = 0
            cmd_pub.publish(twist)
            break
            
        #rospy.loginfo(twist)
        cmd_pub.publish(twist)
        rate.sleep()
    

def main():

    try:
        rospy.init_node('lab02')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
