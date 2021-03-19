#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios



saved_x = []
saved_gt = []
saved_P = []
saved_t = []
count = 0


def getKey():
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

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0, T):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0
        self.gt = 0 #initialize gt
        self.T = T
        self.z_pred = 0
        self.Wkp1 = 0

        self.tk = 0
        self.tkp1 = 0

        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## gt_callback updates the true robot pose ***only to be used for plotting)***
    def gt_callback(self,msg):
        self.gt = msg.pose.pose.position.x

    ## call within run_kf to update the state with the measurement 
    def predict(self, u = 0):
        self.tk = self.tkp1
        self.tkp1 = rospy.get_time()
        dt = self.tkp1 - self.tk

        x_cn = 2.59
        y_cn = 1.21
       

        D_k = y_cn/(y_cn**2 + (x_cn - self.x)**2)
        self.P = 1*self.P*1 + self.Q
        Skp1 = D_k*self.P*D_k + self.R
        self.Wkp1 = self.P*D_k/Skp1
        self.P = self.P - self.Wkp1*Skp1*self.Wkp1
        self.z_pred = math.atan(y_cn/(x_cn-self.x))
        if x_cn < self.x:
            self.z_pred = self.z_pred + math.pi
        self.x = self.x + dt*self.u
#        if not math.isnan(self.phi):
#            self.x = self.x + self.Wkp1*(self.phi-self.z_pred)
        rospy.loginfo(str(180*self.phi/math.pi) + ", ANGLE")
        rospy.loginfo(str(self.x) + ", predicted state")
        return 

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self):
#        self.x = self.x + self.Wkp1*(self.phi-self.z_pred)
        rospy.loginfo(str(self.x) + ", measurement adjusted")
        return

    def run_kf(self):
        current_input = self.u
        current_measurement = self.phi
        
        ## TODO: complete this by completing self.predict() and self.measurement_update()
        self.predict(current_input)
        self.measurement_update()
        
        self.state_pub.publish(str(float(self.x)))

        # for plotting
        global count
        saved_t.append(self.T * count)
        saved_x.append(self.x)
        saved_gt.append(self.gt)
        saved_P.append(self.P)
        count += 1
	


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 1.0 #y distance to tower
        d = 2.25 #x distance to tower (from origin)  
        
        x_0 = 0 #initial state position
        
        Q = 0.005 #process noise covariance
        R = 0.001 #measurement noise covariance
        P_0 = 1e-5 #state covariance
        hz = 20 #frequency of KF loop
        T = 1.0/hz

        kf = KalmanFilter(h, d, x_0, Q, R, P_0,T)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        kf.gt_sub = rospy.Subscriber('odom',Odometry, kf.gt_callback)
        rospy.sleep(1)
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            kf.run_kf()  
            rate.sleep()
            
    except Exception as e:
        # plotting
        fig, ax = plt.subplots()
        ax.plot(saved_t, saved_x, label='est')
        ax.plot(saved_t, saved_gt, label='gt')

        ax.set(xlabel='time (s)', ylabel='x (m)',
            title='Distance')
        ax.legend()
        fig.savefig("x.png")

        fig2, ax2 = plt.subplots()
        ax2.plot(saved_t, saved_P)

        ax2.set(xlabel='time (s)', ylabel='covariance (m^2)',
            title='Covariance')
        fig2.savefig("covariance.png")
        print(e)

    finally:
        rospy.loginfo("goodbye")

