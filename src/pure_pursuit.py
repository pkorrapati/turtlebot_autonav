#!/usr/bin/env python3
import rospy #ROS library for python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image#ROS message format for images
import cv2#OpenCV library
from cv_bridge import CvBridge#Bridge to go between ros and opencv
import numpy as np
import matplotlib.pyplot as plt
import time 
plt.ion()
polynomial = np.polynomial.polynomial.Polynomial

class Sense_Think:
    def __init__(self):
        rospy.init_node('pure_pursuit.py', anonymous=True)
        self.rate = rospy.Rate(60)#Sets refresh rate
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)#Publisher Node
        self.sub = rospy.Subscriber('/camera/image',Image,self.Sense)
        self.bridge = CvBridge()#CvBridge Function
        self.Act = Act()   
    def Sense(self,Image_Stream):
        self.frame = self.bridge.imgmsg_to_cv2(Image_Stream)
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)#Converts to HSV
        self.homography()
        self.Think()
    def Think(self):
        pass
    def homography(self):
        ''' Entity Scaling '''
        # Transforms world to pixels
        H_scale = np.array([[-0.2371,    0.3775,          0.1890],
                            [-0.1575,   -0.00093736,     -0.8607],
                            [-0.0015,    0.0000071814,    0.0012]])

        #Transforms pixels to world frame
        H_scale_inv = np.linalg.inv(H_scale)

        ''' Image Unravel '''
        H_flat = np.array([[-0.00058790,   -0.0008264,    0.3784],
                        [-0.000036335,   -0.0029,        0.8956],
                        [-0.00000005875,-0.000002559,  0.00060561]])




        h,w,c = self.frame.shape

        self.tFrame = cv2.warpPerspective(self.frame, H_flat, (w, h*2))

        # blob_x, blob_y are in inches. X forward, Y to the left
        # mask_x, mask_y are in pixels
        #[blob_x, blob_y, blob_s] = np.dot(H_scale_inv, [mask_x, mask_y, 1])

        #blob_x = blob_x/blob_s
        #blob_y = blob_y/blob_s
            
        cv2.imshow('Original', self.frame)
        cv2.imshow('Transformed', self.tFrame)
        cv2.waitKey(1)
class Act:
    def __init__(self):
        pass
if __name__ == '__main__':
    try:
        tb=Sense_Think()
        rospy.spin()
        rospy.on_shutdown(lf.Shutdown) 
    except rospy.ROSInterruptException:
        pass
