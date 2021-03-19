#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():
    cmd_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist=Twist()
    rate = rospy.Rate(10)
    motionCnt = 0
    rotationCnt = 0
    while not rospy.is_shutdown():
        if motionCnt <= 50:
            twist.linear.x=0.2
            motionCnt += 1
        elif rotationCnt <= 50:
            twist.linear.x=0
            twist.angular.z=0.4*math.pi
            rotationCnt+=1
        else:
            twist.angular.z=0
        rospy.loginfo(twist)
        cmd_pub.publish(twist)
        rate.sleep()
    
    '''
    TODO: complete the publisher function here
    '''
    pass


def main():

    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
