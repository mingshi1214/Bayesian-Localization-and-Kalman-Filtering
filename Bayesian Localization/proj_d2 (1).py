#!/usr/bin/env python
import math
import time
import numpy as np
import re
import sys, select, os
import matplotlib.pyplot as plt

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
        #self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        #self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        #self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.probability = P0 ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.zeros(np.shape(P0))

        self.CurColour = None ##most recent measured colour

        self.error = 0
        self.errori = 0
        self.lastError = 0

        self.h = 0.15
        self.coolDown = 0

        self.colorCnt = 0
    #self.rate = rospy.Rate(50)
 
    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.CurColour = np.array([r,g,b])

    def line_callback(self, msg):
        '''
        TODO: Complete this with your line callback function from lab 3.
        '''
        self.lastError = self.error
        self.error = -int(msg.data) + 320
        if (int(msg.data) != 0):
            self.errori += self.error*self.h
        self.follow_the_line()
        pass


    def follow_the_line(self):
        err = self.error
        erri = self.errori
        errd = self.error-self.lastError
        kp = 0.0055
        ki = 0.0010
        kd = 0.017
        return

    def waitforcolour(self):
        while(1):
            if self.CurColour is not None:
                break

    def measurement_model(self):
        if self.CurColour is None:
            self.waitforcolour()
        prob=np.zeros(11)
        dist=np.zeros(5)
        minIndex = 0

        for i in range(0,5):
            dist[i] = (self.colourCodes[i][0]-self.CurColour[0])**2 + (self.colourCodes[i][1]-self.CurColour[1])**2 + (self.colourCodes[i][2]-self.CurColour[2])**2
            if (dist[i] <= dist[minIndex]):
                minIndex = i
        #Iffy about the logic below
        if minIndex == 0:  #Green
            prob = np.array([0.05,0.6,0.2,0.05,0.05,0.6,0.2,0.05,0.05,0.6,0.2])
        elif minIndex == 1: #Orange
            prob = np.array([0.15,0.05,0.05,0.6,0.6,0.05,0.05,0.6,0.15,0.05,0.05])
        elif minIndex == 2: #Purple
            prob = np.array([0.05,0.2,0.6,0.05,0.05,0.2,0.6,0.05,0.05,0.2,0.6])
        elif minIndex == 3: #Yellow
            prob = np.array([0.65,0.05,0.05,0.2,0.2,0.05,0.05,0.2,0.65,0.05,0.05])
        elif minIndex == 4:
            prob = np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])
        '''
        Measurement model p(z_k | x_k = colour) - given the pixel intensity, what's the probability that  
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.CurColour) 
            and the reference RGB values of each color (self.ColourCodes).
        '''
        return prob

    def statePredict(self,forward):
        self.statePrediction = np.zeros(11)
        for i in range(0,11):
            if forward == 1:
                 self.statePrediction[(i+1)%11] +=  0.85*self.probability[i]
                 self.statePrediction[i] += 0.1*self.probability[i]
                 self.statePrediction[(i-1)%11] += 0.05*self.probability[i]
            elif forward == 0:
                 self.statePrediction[(i+1)%11] += 0.05*self.probability[i]
                 self.statePrediction[i] += 0.9*self.probability[i]
                 self.statePrediction[(i-1)%11] += 0.05*self.probability[i]
            else:
                 self.statePrediction[(i+1)%11] += 0.05*self.probability[i]
                 self.statePrediction[i] += 0.1*self.probability[i]
                 self.statePrediction[(i-1)%11] += 0.85*self.probability[i]
        '''
        TODO: Complete the state prediction function
        '''

    def stateUpdate(self, forward):
        self.statePredict(forward)
        prob = self.measurement_model()
        norm = 0
        for i in range(0,11):
            norm += self.statePrediction[i]*prob[i]
        for i in range(0,11):
            self.probability[i] = self.statePrediction[i]*prob[i]/norm
        
        print(self.probability)

        '''
        TODO: Complete the state update function
        '''      

color_maps = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1] ## current map starting at cell#2 and ending at cell#12
color_codes = [[72, 255, 72], #green
                   [255, 144, 0], #orange
                   [145,145,255], #purple
                   [255, 255, 0], #yellow
                   [133,133,133]] #line

trans_prob_fwd = [0.1,0.9]
trans_prob_back = [0.2,0.8]

bayesian=BayesLoc([1.0/len(color_maps)]*len(color_maps), color_codes, color_maps, trans_prob_back,trans_prob_fwd)

print(bayesian.probability)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[1]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[3]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[0]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[2]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[4]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[0]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[2]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[0]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[1]
bayesian.stateUpdate(0)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[3]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[0]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()

bayesian.CurColour = color_codes[2]
bayesian.stateUpdate(1)
fig = plt.figure()
ax = fig.add_axes([0,0,1,1])
langs = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10']
ax.bar(langs,bayesian.probability)
plt.show()
