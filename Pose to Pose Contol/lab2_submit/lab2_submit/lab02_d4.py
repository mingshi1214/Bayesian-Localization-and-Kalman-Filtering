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
    x=0
    while not rospy.is_shutdown():

        fx=0.25*math.sin(x/0.25)
        fxp=2*math.cos(x/0.25)
        fxpp=-8*math.sin(x/0.25)
        r=0
        w=0
        if fxpp == 0:
           r=0
           w=0
        else:
           r = ((1+fxp**2)**(3/2))/fxpp
           w = speed/r

        twist.linear.x=speed
        twist.angular.z=w
        
        x += speed*math.cos(math.atan(fxp))/500

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
