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

''' General Configuration '''
PLOT_THINGS = False   # This enables plots to be drawn. Default: False
USE_YOLO_CUDA = False # This enables YOLO-CUDA. Default: True

''' Turtlebot Parameters '''
# CRASH_DIST = 0.18      # Actual robot is 0.105 in radius
CRASH_DIST = 0.40      # Actual robot is 0.105 in radius
# TURTLEBOT_WIDTH = 0.23 # Actual robot wheel to wheel width is 0.178
TURTLEBOT_WIDTH = 0.35 # Actual robot wheel to wheel width is 0.178
CRASH_ANGLE = atan2(TURTLEBOT_WIDTH, (2 * CRASH_DIST))

''' LIDAR Parameters '''
LIDAR_INF = 5.0        # LDS-01 LIDAR range is 3.5m max
LIDAR_EDGE_DEPTH = 0.2 # To identify continuous vs discontinuous edges
MIN_EDGE_WIDTH = 3     # Degrees in cylindrical coordinates

# H_Mat = [[-65.5424,    1.5083,  301.0359],
#          [1.3490,  -17.1878,  566.0193],
#          [0.0000,    0.0000,    1.0000]]

''' Entity Scaling '''
H_scale = np.array([[-0.2371,    0.3775,          0.1890],
                    [-0.1575,   -0.00093736,     -0.8607],
                    [-0.0015,    0.0000071814,    0.0012]])

''' Image Unravel '''
H_flat = np.array([[-0.00058790,   -0.0008264,    0.3784],
                   [-0.000036335,   -0.0029,        0.8956],
                   [-0.00000005875,-0.000002559,  0.00060561]])

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
    dc = dists.mean()
    # dc = np.divide(np.multiply(areas, np.dot(dists, 0.5)).sum(), dists.sum())

    return ac, dc

def consecutive(data, stepsize=1):
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)

def cylToCart(angles, dists):
    X = np.multiply(dists, np.cos(angles))
    Y = np.multiply(dists, np.sin(angles))

    return X, Y

def fitLine(X, Y):
    A = np.vstack([X, np.ones(len(X))]).T

    m,c = np.linalg.lstsq(A, Y, rcond=0)[0]
    return [1, -m, -c]

class VisualCortex:
    def __init__(self):
        # Brute Force matcher
        self.orb = cv2.ORB_create(50)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        self.stopSignMemory = cv2.imread('memory/stop.png', 0)           
        self.memoryKP, self.memoryDesc = self.orb.detectAndCompute(self.stopSignMemory, None) 
        
        # ''' Load YOLO START'''
        # labelsPath = os.path.sep.join(["memory", "coco.names"])
        # configPath = os.path.sep.join(["memory", "yolov3.cfg"])
        # weightsPath = os.path.sep.join(["memory", "yolov3.weights"])
        
        # np.random.seed(42)

        # self.LABELS = open(labelsPath).read().strip().split("\n")
        # self.COLORS = np.random.randint(0, 255, size=(len(self.LABELS), 3), dtype="uint8")

        # self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

        # if USE_YOLO_CUDA:
        #     self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        #     self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # self.ln = self.net.getLayerNames()
        # self.ln = [self.ln[i - 1] for i in self.net.getUnconnectedOutLayers()]
        # ''' Load YOLO END'''
        
        rospy.init_node('visual_cortex', anonymous=True)
        
        ''' Subscribers '''
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.analyze)
        self.sub_lidar = rospy.Subscriber('/scan', LaserScan, self.echo)        
        # self.sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.visualize)
        # self.sub_image = rospy.Subscriber('/camera/image', Image, self.visualize)        

        ''' Publishers '''
        self.pub_motion = rospy.Publisher('/cmd_vel_obstacle', Twist, queue_size=10)

        self.isAwake = False    
        self.newEcho = False
        self.refresh = False
        
        self.fAngle = radians(0)
        self.rAngle = self.fAngle - radians(90)
        self.lAngle = self.fAngle + radians(90)

        # Range of front crash angles
        # crash is largest angle of triangle at smallest distance
                
        self.fPlus = CRASH_ANGLE
        self.fMins = CRASH_ANGLE

        self.frontHalfAngles = np.array([])
        self.frontHalfRanges = np.array([])

        self.vel_msg = Twist()
        self.bridge = CvBridge()

        self.cv_image = np.zeros([5,5])

        if PLOT_THINGS: 
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(132)
            self.bx = self.fig.add_subplot(133, projection='polar')
            self.cx = self.fig.add_subplot(131).imshow(self.cv_image)
            
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
                     
        self.pulRate = data.rate

        if self.newEcho:
            self.ac, self.dc = getCentroid(self.frontHalfAngles, self.frontHalfRanges)

            cornerIndex = np.where(self.rangeDiff > LIDAR_EDGE_DEPTH)[0] + 1
            # cInd1 = np.sort(np.unique(np.append(cornerIndex, [0, len(self.frontHalfRanges)])))
            # print(cornerIndex)

            # cornerIndex = cornerIndex[np.where(np.diff(cornerIndex) > MIN_EDGE_WIDTH)[0] + 1] # Filters out anomalies < 3 lidar points

            # print(cornerIndex)

            # if len(cornerIndex) > 0 and (len(self.frontHalfRanges) - cornerIndex[-1]) <= MIN_EDGE_WIDTH:
            #     cornerIndex= cornerIndex[:-1] # Filter out anomalies

            edges = np.split(self.rangeDiff, cornerIndex)
            
            strCorners=np.array([])
            endCorners=np.array([])

            if len(cornerIndex) > 0:
                strCorners = np.append([0], cornerIndex)
                endCorners = np.append(cornerIndex -1, [len(self.frontHalfAngles)])

            # print(strCorners)
            # print(endCorners)

            ''' If obstacles are too close, dynamic thresh will not find a way out'''
            # DYNAMIC_THRESHOLD = 2

            segments = []

            # Edges as a list of start and end index
            for i in range(len(cornerIndex)):
                if i==0:                    
                    segments = [[0,cornerIndex[i]]]
                else:
                    segments = np.vstack((segments, [cornerIndex[i-1], cornerIndex[i]]))
            
            segments = np.vstack((segments, [cornerIndex[-1], len(self.frontHalfAngles) -1 ]))

            segCents = []

            for segment in segments:                                           
                acS, dcS = getCentroid(self.frontHalfAngles[segment[0]:segment[1]], self.frontHalfRanges[segment[0]:segment[1]])
                
                if len(segCents) == 0:
                    segCents = [acS, dcS]
                else:
                    segCents = np.vstack((segCents, [acS, dcS]))
            
            targI = np.argmax(segCents[:,1])
            targA = np.argmax(np.multiply(segCents[:,1], segCents[:,0]))

            if PLOT_THINGS:     
                self.distaPlot.set_xdata(self.frontHalfAngles)
                self.distaPlot.set_ydata(self.frontHalfRanges)

                self.distbPlot.set_xdata(self.frontHalfAngles)
                self.distbPlot.set_ydata(self.frontHalfRanges)   
                
                self.distaPlot2.set_xdata(self.ac)
                self.distaPlot2.set_ydata(self.dc) 

                self.distbPlot2.set_xdata(self.ac)
                self.distbPlot2.set_ydata(self.dc)   

                # self.distaPlot2.set_xdata(segCents[:,0])
                # self.distaPlot2.set_ydata(segCents[:,1]) 

                # self.distbPlot2.set_xdata(segCents[:,0])
                # self.distbPlot2.set_ydata(segCents[:,1])   

                # self.obstaPlot.set_xdata(self.frontHalfAngles[:-1])
                # self.obstaPlot.set_ydata(self.rangeDiff)

                # self.obstbPlot.set_xdata(self.frontHalfAngles[:-1])
                # self.obstbPlot.set_ydata(self.rangeDiff)

                self.edgeaPlot.set_xdata(self.frontHalfAngles[cornerIndex])
                self.edgeaPlot.set_ydata(self.frontHalfRanges[cornerIndex])

                self.edgebPlot.set_xdata(self.frontHalfAngles[cornerIndex])
                self.edgebPlot.set_ydata(self.frontHalfRanges[cornerIndex])

            # print(np.min(self.crashScan))

            if np.min(self.crashScan) > CRASH_DIST:
                self.vel_msg.linear.x = 0.15                    
                # self.vel_msg.angular.z = -0.02 * self.ac #segCents[targI, 0]
                # self.vel_msg.angular.z = 0.2 * self.ac #segCents[targI, 0]
                self.vel_msg.angular.z = 1.6 * self.ac #segCents[targI, 0]
                # self.vel_msg.angular.z = 0.09 * self.ac #segCents[targI, 0]
                # self.vel_msg.angular.z = 0.12 * self.ac #segCents[targI, 0]
            else:
                # print('crash')
                self.vel_msg.linear.x = 0
                # self.vel_msg.angular.z = 0.4 * segCents[targI, 0]
                self.vel_msg.angular.z = 0.1

            self.newEcho = False
            self.refresh = True            


        self.pub_motion.publish(self.vel_msg)

    def visualize(self, data):         
        # print('image')       
        try:
            # ballLower = (0, 47, 0)
            # ballUpper = (86, 255,255)
            
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # c_img = cv_image[230:, :] if cv_image.shape[0] >= 233 else cv_image

            # # processed = cv2.GaussianBlur(cv_image, (11, 11), 0)
            # processed = cv2.GaussianBlur(c_img, (11, 11), 0)        

            # # processed = cv2.GaussianBlur(cv_image[395:466, :], (11, 11), 0)        
            # processed = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)

            # lineMask = cv2.inRange(processed, ballLower, ballUpper)
            # lineMask = cv2.dilate(lineMask, None, iterations=2)            
            # lineMask = cv2.erode(lineMask, None, iterations=2)

            # lIm, lineContours = cv2.findContours(lineMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)            
            
            # tg_x = 160
            # # tg_y = dcB["m01"] / dcB["m00"]

            # if len(lIm):
            #     line = max(lIm, key=cv2.contourArea)            
            #     cv2.drawContours(c_img, line, -1, (0,255,0))

            #     dcB = cv2.moments(line)
            
            #     if dcB["m00"] != 0:       
            #         tg_x = dcB["m10"] / dcB["m00"]
            #         # tg_y = dcB["m01"] / dcB["m00"]             
            #         # cv2.circle(c_img, (int(tg_x), int(tg_y)), int(5), (0,255,255), 2)
            
            # if np.min(self.crashScan) > CRASH_DIST:
            #     self.vel_msg.linear.x = 0.1
            #     self.vel_msg.angular.z = 0.003 * (160 - tg_x)
            #     # self.vel_msg.angular.z = 0.008 * (160 - tg_x)
            
            # self.pub_motion.publish(self.vel_msg)

            # cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "rgb8")            
            
            # kp, desc = self.orb.detectAndCompute(cv_image, None)  
            # matches = self.bf.match(self.memoryDesc, desc)
            
            # cv_image = cv2.drawMatches(self.stopSignMemory, self.memoryKP, cv_image, kp, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

            # # cv_image = cv2.drawKeypoints(cv_image, kp, None, color=(0,255,0), flags=0)


            # (H, W) = cv_image.shape[:2]
            
            # blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
            # self.net.setInput(blob)            
            # layerOutputs = self.net.forward(self.ln)       
            
            # boxes = []
            # confidences = []
            # classIDs = []

            # # loop over each of the layer outputs
            # for output in layerOutputs:
            #     # loop over each of the detections
            #     for detection in output:
            #         # extract the class ID and confidence (i.e., probability)
            #         # of the current object detection
            #         scores = detection[5:]
            #         classID = np.argmax(scores)
            #         confidence = scores[classID]

            #         # filter out weak predictions by ensuring the detected
            #         # probability is greater than the minimum probability
            #         if confidence > 0.5:
            #             # scale the bounding box coordinates back relative to
            #             # the size of the image, keeping in mind that YOLO
            #             # actually returns the center (x, y)-coordinates of
            #             # the bounding box followed by the boxes' width and
            #             # height
            #             box = detection[0:4] * np.array([W, H, W, H])
            #             (centerX, centerY, width, height) = box.astype("int")

            #             # use the center (x, y)-coordinates to derive the top
            #             # and and left corner of the bounding box
            #             x = int(centerX - (width / 2))
            #             y = int(centerY - (height / 2))

            #             # update our list of bounding box coordinates,
            #             # confidences, and class IDs
            #             boxes.append([x, y, int(width), int(height)])
            #             confidences.append(float(confidence))
            #             classIDs.append(classID)

            # # apply non-maxima suppression to suppress weak, overlapping
            # # bounding boxes
            # idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)

            # # ensure at least one detection exists
            # if len(idxs) > 0:
            #     # loop over the indexes we are keeping
            #     for i in idxs.flatten():
            #         # extract the bounding box coordinates
            #         (x, y) = (boxes[i][0], boxes[i][1])
            #         (w, h) = (boxes[i][2], boxes[i][3])

            #         # draw a bounding box rectangle and label on the frame
            #         color = [int(c) for c in self.COLORS[classIDs[i]]]
            #         cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
            #         # text = "{}: {:.4f}".format(self.LABELS[classIDs[i]],	confidences[i])
            #         text = self.LABELS[classIDs[i]]
            #         cv2.putText(cv_image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Send labeled image 
            # image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            # self.pub_image.publish(image_message)

            # if plotThings:
            self.cv_image = cv_image.copy()                                                
                # self.cx.imshow(cv_image)

        except CvBridgeError as e:
            print(e)
        
        # cv2.imshow("Image window", cv_image)
    
    def echo(self, data):        
        incAngle = data.angle_increment
        minAngle = data.angle_min
                
        ranges = np.array(data.ranges)
            
        # Clean up INF in data
        ranges[np.where(ranges == np.inf)] = LIDAR_INF
        ranges[np.where(ranges == 0)] = LIDAR_INF
        ranges[np.where(ranges == np.NaN)] = LIDAR_INF

        # Indices of Left End and Right End of Region of Interest
        # use these only with data.ranges                
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

        # front scan
        self.crashScan = extractRanges(ranges, fIndx, self.fMinsIndx, self.fPlusIndx)
        self.crashRegion = np.linspace(self.fAngle - self.fMins, self.fAngle + self.fPlus, num=len(self.crashScan))

        # print(self.crashScan)

        # X, Y = cylToCart(self.crashRegion, self.crashScan)
        # wall_eq = fitLine(X, Y)

        # print(degrees((pi/2)- atan(-wall_eq[1])))

        self.rangeDiff = np.abs(np.diff(self.frontHalfRanges))
        # np.hstack((np.abs(np.subtract(self.frontHalfRanges[1:], self.frontHalfRanges[0:frontHalfCount-1])), [0]))

        # Zero front index in self.frontHalfRanges
        # self.zIndx = -rIndx -1

        # counter = (counter + 1) % 5
        self.newEcho = True        

if __name__ == '__main__':
    try:
        vNode = VisualCortex()

        while not rospy.is_shutdown():
            if vNode.refresh and PLOT_THINGS:
                vNode.cx.set_data(vNode.cv_image)
                vNode.cx.autoscale()

                vNode.fig.canvas.draw()
                vNode.fig.canvas.flush_events()

                plt.subplots_adjust(bottom=0.1, top=0.9)
                vNode.refresh = False                
        
    except rospy.ROSInterruptException:
        pass