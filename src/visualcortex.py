#!/usr/bin/env python3

import numpy as np
import cv2

import rospy

# import imagezmq
# import argparse
# import time
# import os

from math import pi, radians, atan2

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from final_prj.msg import Pulse, MotionCmd

import tf2_ros
import matplotlib.pyplot as plt
plt.ion()

plotThings = True

crashDist = 0.3
tbWidth = 0.2 # TODO Put actual value

# Remaps (-pi/2, pi/2) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

def extractRanges(ranges, center, Mins, Plus):
    if (center - Mins) < 0 and (center + Plus) > 0 :
        return np.hstack((ranges[(center - Mins):], ranges[:(center + Plus)]))
    else:
        return np.array(ranges[(center - Mins):(center + Plus)])
    
def getCentroid(angles, dists):   
    angleStep = 1

    areas = np.dot(dists, angleStep)

    ac = np.divide(np.multiply(areas, angles).sum(), np.abs(angles).sum())
    dc = np.divide(np.multiply(areas, np.dot(dists, 0.5)).sum(), dists.sum())

    return ac, dc

class VisualCortex:
    def __init__(self):        
        rospy.init_node('visual_cortex', anonymous=True)

        # Subscribers
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.analyze)
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.echo)
        self.sub_image = rospy.Subscriber('/image', CompressedImage, self.visualize)        

        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        self.isAwake = False    
        self.newEcho = False
        self.refresh = False

        self.fAngle = radians(0)
        self.rAngle = self.fAngle - radians(90)
        self.lAngle = self.fAngle + radians(90)

        # Range of front crash angles
        # crash is largest angle of triangle at smallest distance
        crashAngle = atan2(tbWidth, (2*crashDist))
        
        self.fPlus = crashAngle
        self.fMins = crashAngle

        self.frontHalfAngles = np.array([])
        self.frontHalfRanges = np.array([])

        self.vel_msg = Twist()

        if plotThings:            
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(121)
            self.bx = self.fig.add_subplot(122, projection='polar')
            
            self.ax.invert_xaxis()
            self.bx.set_theta_zero_location("N")            
            
            self.distaPlot, = self.ax.plot([self.rAngle, self.lAngle], [0,4])
            self.distaPlot2, = self.ax.plot([self.rAngle, self.lAngle], [0,4], 'rx')
            self.obstaPlot, = self.ax.plot([self.rAngle, self.lAngle], [0,4], 'y.')
            self.edgeaPlot, = self.ax.plot([self.rAngle, self.lAngle], [0,4], 'go')
            
            self.distbPlot, = self.bx.plot([0, 2*pi], [0,4])
            self.distbPlot2, = self.bx.plot([0, 2*pi], [0,4], 'rx')
            self.obstbPlot, = self.bx.plot([0, 2*pi], [0,4], 'y.')
            self.edgebPlot, = self.bx.plot([0, 2*pi], [0,4], 'go')

    def analyze(self, data):                
        # self.pub_motion.publish(self.vel)     
        self.pulRate = data.rate

        if self.newEcho:
            self.ac, self.dc = getCentroid(self.frontHalfAngles, self.frontHalfRanges)

            # Indexes of edges
            edges = np.where(self.rangeDiff > 0.15)[0]
            edges = np.subtract(edges, -1) if len(edges) > 0 else edges
            segments = []

            # Edges as a list of start and end index
            for i in range(len(edges)):
                if i==0:                    
                    segments = [[0,edges[i]]]
                else:
                    segments = np.vstack((segments, [edges[i-1], edges[i]]))
            
            segments = np.vstack((segments, [edges[-1], len(self.frontHalfAngles) -1 ]))

            segCents = []

            for segment in segments:                                           
                acS, dcS = getCentroid(self.frontHalfAngles[segment[0]:segment[1]], self.frontHalfRanges[segment[0]:segment[1]])
                
                if len(segCents) == 0:
                    segCents = [acS, dcS]
                else:
                    segCents = np.vstack((segCents, [acS, dcS]))
            
            targI = np.argmax(segCents[:,1])

            self.distaPlot.set_xdata(self.frontHalfAngles)
            self.distaPlot.set_ydata(self.frontHalfRanges)

            self.distbPlot.set_xdata(self.frontHalfAngles)
            self.distbPlot.set_ydata(self.frontHalfRanges)   
            
            self.distaPlot2.set_xdata(segCents[:,0])
            self.distaPlot2.set_ydata(segCents[:,1]) 

            self.distbPlot2.set_xdata(segCents[:,0])
            self.distbPlot2.set_ydata(segCents[:,1])   

            self.obstaPlot.set_xdata(self.frontHalfAngles)
            self.obstaPlot.set_ydata(self.rangeDiff)

            self.obstbPlot.set_xdata(self.frontHalfAngles)
            self.obstbPlot.set_ydata(self.rangeDiff)

            self.edgeaPlot.set_xdata(self.frontHalfAngles[edges])
            self.edgeaPlot.set_ydata(self.frontHalfRanges[edges])

            self.edgebPlot.set_xdata(self.frontHalfAngles[edges])
            self.edgebPlot.set_ydata(self.frontHalfRanges[edges])

            # print(self.crashScan)

            if np.min(self.crashScan) > 0.3:
                self.vel_msg.linear.x = 0.15                    
                self.vel_msg.angular.z = -0.02 * segCents[targI, 0]
            else:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0.1

            self.newEcho = False
            self.refresh = True            

        self.pub_motion.publish(self.vel_msg)

    def visualize(self, data):
        pass
    
    def echo(self, data):
        incAngle = data.angle_increment
        minAngle = data.angle_min
                
        ranges = np.array(data.ranges)
        count = len(ranges)
        
        # Clean up inf in data
        ranges[np.where(ranges == np.inf)] = 10.0

        # Indices of Left End and Right End of Region of Interest
        # use these only with data.ranges

        # TODO : During calibration, we might get offsets for front, left and right cases
        # update code to handle + and - cases of each angles. Especially front
        fIndx = round((self.fAngle - minAngle)/incAngle) # Index of heading
        lIndx = round((self.lAngle - minAngle)/incAngle) # Index of left        
        rIndx = round((self.rAngle - minAngle)/incAngle) # Index of right        
        
        # Front Half Scan Region
        self.frontHalfRanges = np.hstack((ranges[rIndx:], ranges[:lIndx + 1]))
        frontHalfCount = len(self.frontHalfRanges) #lIndx - rIndx

        # Generate continuous angles from Right to Left
        if len(self.frontHalfAngles) != frontHalfCount:            
            self.frontHalfAngles = np.linspace(self.rAngle, self.lAngle, num=frontHalfCount)
        
        # ------
        # Tested till here
        # ------

        # Front crash region
        self.fPlusIndx = round(self.fPlus/incAngle)
        self.fMinsIndx = round(self.fMins/incAngle)

        # print(fIndx)
        # print(self.fPlusIndx)
        # print(self.fMinsIndx)

        # front scan
        self.crashScan = extractRanges(ranges, fIndx, self.fMinsIndx, self.fPlusIndx)
        self.crashRegion = np.linspace(self.fAngle - self.fMins, self.fAngle + self.fPlus, num=len(self.crashScan))

        self.rangeDiff = np.hstack((np.abs(np.subtract(self.frontHalfRanges[1:], self.frontHalfRanges[0:frontHalfCount-1])), [0]))

        # Zero front index in self.frontHalfRanges
        # self.zIndx = -rIndx -1

        # counter = (counter + 1) % 5
        self.newEcho = True

if __name__ == '__main__':
    try:
        vNode = VisualCortex()

        while not rospy.is_shutdown():
            if vNode.refresh:
                vNode.fig.canvas.draw()
                vNode.fig.canvas.flush_events()

                plt.subplots_adjust(bottom=0.1, top=0.9)
                vNode.refresh = False
        
    except rospy.ROSInterruptException:
        pass
