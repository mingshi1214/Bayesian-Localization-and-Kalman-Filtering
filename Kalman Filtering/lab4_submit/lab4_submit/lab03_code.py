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
        self.state = rospy.Subscriber('state',String,self.state_callback,queue_size=1)
        self.error = 0
        self.errori = 0
        self.lastError = 0
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.h = 0.15
        self.rate = rospy.Rate(50)
        self.x = 0
        self.loc1 = False
        self.loc2 = False
        self.loc3 = False


        self._cmd_pub.publish(self.twist)
    def camera_callback(self, data):
        #rospy.loginfo(data)
        self.lastError = self.error
        self.error = -int(data.data) + 320
        if (int(data.data) != 0):
            self.errori += self.error*self.h
        self.follow_the_line()
        pass

    def state_callback(self, data):
        x = float(data.data)
        
        if x >= 5.5 and self.loc3 == False:
            print("Stopping at 3")
            self.twist.linear.x = 0
            rospy.sleep(4)
            self.loc3 = True
        elif x >= 3.2 and self.loc2 == False:
            print("Stopping at 2")
            self.twist.linear.x = 0
            rospy.sleep(4)
            self.loc2 = True
        elif x >= 2.1 and self.loc1 == False:
            print("Stopping at 1")
            self.twist.linear.x = 0
            rospy.sleep(4)
            self.loc1 = True
        self.twist.linear.x = 0.1

    def follow_the_line(self):
        err = self.error
        erri = self.errori
        errd = self.error-self.lastError
        rospy.loginfo("Errp = " + str(err) + ", Erri = " + str(erri) + ", Errd = " + str(errd))
        kp = 0.0028
        ki = 0.0001
        kd = 0.010
        self.twist.angular.z = kp*err + ki*erri + kd*errd
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


