#!/usr/bin/env python3

import numpy as np
import cv2

import os
import rospy

# import imagezmq

from math import pi, radians, atan2, atan, degrees

from turtlebot_autonav.msg import Pulse, MotionCmd
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, CompressedImage

from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import matplotlib.pyplot as plt
plt.ion()

''' Image Unravel '''
# H_flat = np.array([[-0.00058790,   -0.0008264,    0.3784],
#                    [-0.000036335,   -0.0029,        0.8956],
#                    [-0.00000005875,-0.000002559,  0.00060561]])

# H_flat = np.array([[-0.0014,        -0.0020,        0.4534],
#                    [-0.000044831,   -0.0051,        0.8913],
#                    [-0.00000028155,-0.000012263,    0.0015]])

H_flat = np.array([[-0.0012,        -0.0017,        0.3784],
                   [-0.000044831,   -0.0051,        0.9256],
                   [-0.000000235,-0.00001023,       0.0012]])

class VisualCortex:
    def __init__(self):        
        rospy.init_node('visual_cortex', anonymous=True)
        
        ''' Subscribers '''
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.analyze)

        # self.sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.visualize)
        self.sub_image = rospy.Subscriber('/camera/image', Image, self.visualize)        

        ''' Publishers '''
        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.isAwake = False    
        self.newEcho = False
        self.refresh = False
                
        self.vel_msg = Twist()
        self.bridge = CvBridge()

        self.cv_image = np.zeros([5,5])
        self.cv_image_T = np.zeros([5,5])

        self.fig = plt.figure()
            
        self.cx = self.fig.add_subplot(121).imshow(self.cv_image)
        self.dx = self.fig.add_subplot(122).imshow(self.cv_image_T)

    def analyze(self, data):       
                     
        self.pulRate = data.rate

        # if self.newEcho:
        
        
        #     self.newEcho = False
        #     self.refresh = True            

        # self.pub_motion.publish(self.vel_msg)

    def visualize(self, data):         
        # print('image')       
        try:
            # ballLower = (0, 47, 0)
            # ballUpper = (86, 255,255)
            
            # ballLower = (0, 140, 80)
            # ballUpper = (180, 162, 180)

            ballLower = (0, 120, 80)
            ballUpper = (180, 182, 180)

            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            
            c_img = cv_image[230:, :] if cv_image.shape[0] >= 233 else cv_image
            c_img = cv_image
            
            processed = cv2.GaussianBlur(c_img, (11, 11), 0)        
            # processed = cv2.GaussianBlur(cv_image, (11, 11), 0)        

            # # processed = cv2.GaussianBlur(cv_image[395:466, :], (11, 11), 0)        
            processed = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)

            lineMask = cv2.inRange(processed, ballLower, ballUpper)
            lineMask = cv2.dilate(lineMask, None, iterations=2)            
            lineMask = cv2.erode(lineMask, None, iterations=2)

            lIm, lineContours = cv2.findContours(lineMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)            
            
            # tg_x = 160
            # # tg_y = dcB["m01"] / dcB["m00"]

            if len(lIm):
                line = max(lIm, key=cv2.contourArea)            
                cv2.drawContours(c_img, line, -1, (0,255,0))

                dcB = cv2.moments(line)
            
                if dcB["m00"] != 0:       
                    tg_x = dcB["m10"] / dcB["m00"]
                    tg_y = dcB["m01"] / dcB["m00"]             
                    cv2.circle(c_img, (int(tg_x), int(tg_y)), int(5), (0,255,255), 2)
            
            # # if np.min(self.crashScan) > CRASH_DIST:
            # #     self.vel_msg.linear.x = 0.1
            # #     self.vel_msg.angular.z = 0.003 * (160 - tg_x)
            # #     # self.vel_msg.angular.z = 0.008 * (160 - tg_x)
            
            # # self.pub_motion.publish(self.vel_msg)

            # # cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "rgb8")            
            
            h,w,c = cv_image.shape

            self.cv_image = c_img #cv_image   
            self.cv_image_T = cv2.warpPerspective(cv_image, H_flat, (w,h))
            
            vNode.refresh = True

        except CvBridgeError as e:
            print(e)
           
   
if __name__ == '__main__':
    try:
        vNode = VisualCortex()

        while not rospy.is_shutdown():
            if vNode.refresh:
                vNode.cx.set_data(vNode.cv_image)
                vNode.cx.autoscale()
                vNode.dx.set_data(vNode.cv_image_T)
                vNode.dx.autoscale()

                vNode.fig.canvas.draw()
                vNode.fig.canvas.flush_events()

                plt.subplots_adjust(bottom=0.1, top=0.9)
                vNode.refresh = False                
        
    except rospy.ROSInterruptException:
        pass