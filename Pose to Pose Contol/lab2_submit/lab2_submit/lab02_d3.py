#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def publisher_node():
    cmd_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    speed = 0.2
    radius=0.25
    cnt = [0,0,0,0]
    twist = Twist()
    distance1 = 1.85
    distance2 = radius*(math.pi/2)
    distance3 = 0.10
    distance4 = radius*(math.pi/4)
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        if cnt[0] <= 500*distance1/speed:
            twist.linear.x=speed
            twist.angular.z=0
            cnt[0]+=1
        elif cnt[1] <= 500*distance2/speed:
            twist.linear.x=speed
            twist.angular.z=speed/radius
            cnt[1] += 1
        elif cnt[2] <= 500*distance3/speed:
            twist.linear.x=speed
            twist.angular.z = 0
            cnt[2] += 1
        elif cnt[3] <= 500*distance4/speed:
            twist.linear.x=speed
            twist.angular.z=speed/radius
            cnt[3]+=1
        else:
            twist.angular.z = 0
            twist.linear.x = 0
            
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
