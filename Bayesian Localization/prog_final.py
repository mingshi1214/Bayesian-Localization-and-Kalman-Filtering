#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios


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


class BayesLoc:

    def __init__(self, P0, colourCodes, colourMap, transProbBack, transProbForward):
        self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.probability = P0  ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.zeros(np.shape(P0))

        self.CurColour = None  ##most recent measured colour

        self.error = 0
        self.errori = 0
        self.lastError = 0
        self.twist = Twist()
        self.twist.linear.x = 0.10
        self.twist.angular.z = 0
        self.h = 0.15
        self.coolDown = 0

        self.colorCnt = 0
        self.rate = rospy.Rate(50)

    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:', '').replace('b:', '').replace('g:', '').replace(' ', '')
        r, g, b = rgb.split(',')
        r, g, b = (float(r), float(g), float(b))
        self.CurColour = np.array([r, g, b])

    def line_callback(self, msg):
        self.lastError = self.error
        self.error = -int(msg.data) + 320
        if (int(msg.data) != 0):
            self.errori += self.error * self.h
        self.follow_the_line()
        pass

    def follow_the_line(self):
        err = self.error
        erri = self.errori
        errd = self.error - self.lastError
        kp = 0.0055
        ki = 0.001
        kd = 0.017
        if (self.CurColour[0] >= 210 and self.CurColour[1] >= 210 and self.CurColour[2] >= 210):
            if (rospy.get_time() - self.coolDown >= 9):
                self.twist.angular.z = kp * err + ki * erri + kd * errd
        else:
            self.twist.angular.z = 0
            self.cmd_pub.publish(self.twist)
            if (rospy.get_time() - self.coolDown >= 13):
                rospy.loginfo("Colour Detected!!! Stopping line following temporarily")
                self.coolDown = rospy.get_time()
                self.twist.linear.x = 0
                rospy.sleep(3)
                self.cmd_pub.publish(self.twist)
                rospy.sleep(3)
                self.colorCnt = self.colorCnt + 1
                self.stateUpdate()
                maxIndex = 0
                for i in range(0, 11):
                    if self.probability[i] >= self.probability[maxIndex]:
                        maxIndex = i
                rospy.loginfo("At position: " + str(maxIndex + 1) + ", with prob " + str(self.probability[maxIndex]))
                if maxIndex in (7, 9, 0) and self.probability[maxIndex] > .92:
                    self.deliver()
                    self.coolDown += 12
                self.twist.linear.x = 0.10
        self.cmd_pub.publish(self.twist)
        self.rate.sleep()
        return

    def deliver(self):
        self.twist.linear.x = 0
        self.twist.angular.z = math.pi / 12
        self.cmd_pub.publish(self.twist)
        rospy.sleep(6)
        self.twist.angular.z = 0
        self.twist.angular.z = -math.pi / 12
        self.cmd_pub.publish(self.twist)
        rospy.sleep(6)
        self.twist.angular.z = 0
        self.twist.linear.x = 0.1
        self.cmd_pub.publish(self.twist)

    def waitforcolour(self):
        while (1):
            if self.CurColour is not None:
                break

    def measurement_model(self):
        if self.CurColour is None:
            self.waitforcolour()
        prob = np.zeros(11)
        dist = np.zeros(4)
        minIndex = 0

        for i in range(0, 4):
            dist[i] = (self.colourCodes[i][0] - self.CurColour[0]) ** 2 + (
                        self.colourCodes[i][1] - self.CurColour[1]) ** 2 + (
                                  self.colourCodes[i][2] - self.CurColour[2]) ** 2
            if (dist[i] <= dist[minIndex]):
                minIndex = i
        rospy.loginfo(minIndex)

        for i in range(0, 4):
            if (i == minIndex):
                prob[i] = 0.88
            else:
                prob[i] = 0.04

        return prob

    def statePredict(self, forward):
        self.statePrediction = np.zeros(11)
        for i in range(0, 11):
            if forward == 1:
                self.statePrediction[(i + 1) % 11] += 0.95 * self.probability[i]
                self.statePrediction[i] += 0.05 * self.probability[i]
                self.statePrediction[(i - 1) % 11] += 0 * self.probability[i]
            elif forward == 0:
                self.statePrediction[(i + 1) % 11] += 0.05 * self.probability[i]
                self.statePrediction[i] += 0.9 * self.probability[i]
                self.statePrediction[(i - 1) % 11] += 0.05 * self.probability[i]
            else:
                self.statePrediction[(i + 1) % 11] += 0.05 * self.probability[i]
                self.statePrediction[i] += 0.1 * self.probability[i]
                self.statePrediction[(i - 1) % 11] += 0.85 * self.probability[i]

    def stateUpdate(self):
        self.statePredict(1)
        prob = self.measurement_model()
        norm = 0
        for i in range(0, 11):
            norm += self.statePrediction[i] * prob[self.colourMap[i]]
        for i in range(0, 11):
            self.probability[i] = self.statePrediction[i] * prob[self.colourMap[i]] / norm


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # 0: Green, 1: Orange, 2: Purple, 3: Yellow, 4: Line
    color_maps = [3, 1, 0, 3, 1, 2, 0, 2, 1, 0, 3]  ## current map starting at cell#2 and ending at cell#12
    color_codes = [[72, 255, 72],  # green
                   [255, 174, 0],  # orange
                   [145, 145, 255],  # purple
                   [255, 255, 0],  # yellow
                   [133, 133, 133]]  # line

    trans_prob_fwd = [0.1, 0.9]
    trans_prob_back = [0.2, 0.8]

    rospy.init_node('final_project')
    bayesian = BayesLoc([1.0 / len(color_maps)] * len(color_maps), color_codes, color_maps, trans_prob_back,
                        trans_prob_fwd)
    prob = []
    rospy.sleep(0.5)
    state_count = 0

    prev_state = None
    try:

        while (1):
            key = getKey()
            if (key == '\x03'):
                rospy.loginfo('Finished!')
                rospy.loginfo(prob)
                break


    except Exception as e:
        print("comm failed:{}".format(e))

    finally:
        rospy.loginfo(bayesian.probability)
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        cmd_publisher.publish(twist)
