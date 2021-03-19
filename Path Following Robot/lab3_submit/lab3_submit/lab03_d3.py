#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey(): #you can ignore this function. It's for stopping the robot when press 'Ctrl+C'
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



class PIDcontrol():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)
        self.error = 0
        self.errori = 0
        self.twist = Twist()
        self.twist.linear.x = 0.20
        self.twist.angular.z = 0
        self.h = 0.15
        self.rate = rospy.Rate(50)

        self._cmd_pub.publish(self.twist)
    def camera_callback(self, data):
        rospy.loginfo(data)
        self.error = -int(data.data) + 320
        if (int(data.data) != 0):
            self.errori += self.error*self.h
        self.follow_the_line()
        pass


    def follow_the_line(self):
        err = self.error
        erri = self.errori
        kp = 0.006
        ki = 0.0015
        self.twist.angular.z = kp*err + ki*erri
        self._cmd_pub.publish(self.twist)
        self.rate.sleep()
        pass


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab3')
    PID = PIDcontrol()
    try:
        while(1):
            key = getKey()
            PID.follow_the_line()
            if (key == '\x03'): #stop the robot when exit the program
                break
    except rospy.ROSInterruptException:
        print("comm failed")


