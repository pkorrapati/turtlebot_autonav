#!/usr/bin/env python3
import rospy #ROS library for python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan#ROS message format for images
import cv2#OpenCV library
from cv_bridge import CvBridge#Bridge to go between ros and opencv
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
import time 
plt.ion()
LIDAR_INF = 5
##Global Variables##
found_line = False
on_line = False
stop_sign = False
stopping = True
# Scaling Function
def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]
class Controller:
    def __init__(self):
        #self.input = input
        ##Error Parameters##
        self.errorSignal=0
        self.errorSignal_1=0
        self.errorSignal_2=0
        self.TARGET = 90
        ##GAINS##
        P_GAIN = 5
        I_GAIN = .4
        D_GAIN = .8
        self.K_ONE = P_GAIN + I_GAIN + D_GAIN#Gains for discrete PID
        self.K_TWO = -P_GAIN + -2 * D_GAIN
        self.K_THREE = D_GAIN
    def steer(self,input):
        output=self.errorSignal * self.K_ONE + self.errorSignal_1 * self.K_TWO + self.errorSignal_2 * self.K_THREE#output signal for discrete PID
        self.errorSignal_2 = self.errorSignal_1#Errors for discrete PID
        self.errorSignal_1 = self.errorSignal
        self.errorSignal = input - self.TARGET 
        #print("Error:")
        #print(self.errorSignal)
        return output
class Turtlebot_Movement:
    def __init__(self):
        rospy.init_node('navigator.py', anonymous=True)
        self.rate = rospy.Rate(60)#Sets refresh rate
        self.pub = rospy.Publisher('/cmd_vel_camera',Twist, queue_size=10)#Publisher Node
        self.pub_helm = rospy.Publisher('/helm/line',Bool, queue_size=10)#Publisher Node
        self.sub = rospy.Subscriber('/camera/image',Image,self.Move)
        #self.sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.Move)
        self.sub_stop = rospy.Subscriber('/parrot/stop',Bool,self.Flag_Setter)
        self.sub_scan = rospy.Subscriber('/scan',LaserScan,self.Stopper)
        self.bridge = CvBridge()#CvBridge Function
        self.short_angle_controller = Controller()
        self.mid_angle_controller = Controller()
        self.long_angle_controller = Controller()
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.6
        self.vel_msg.angular.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        #self.pub.publish(self.vel_msg)
        self.not_stopped = True
    def Move(self, image_stream):
        self.Frame_Slicer(image_stream)
        self.Path_Planner()
        if found_line:
            self.pub_helm.publish(True)
            self.pub.publish(self.vel_msg)
    def Path_Planner(self):
        height, width, channels_dont_use = self.current_frame.shape
        
        x_coords = [0,scale(self.bottom_centroid_x,(0,width),(-50,50)),scale(self.middle_centroid_x,(0,width),(-50,50)),scale(self.top_centroid_x,(0,width),(-50,50))]
        y_coords = [0,scale(self.bottom_centroid_y,(0,height),(0,100)),scale(self.middle_centroid_y,(0,height),(0,100)),scale(self.top_centroid_x,(0,height),(0,100))]
        self.path_angle=np.rad2deg(np.arctan2(y_coords,x_coords))
        z_msg = self.Angle()
        self.vel_msg.linear.x=0.06
        self.vel_msg.angular.z=z_msg
        ##Heuristic for recovery##
        if self.missing_lower and on_line:
            self.vel_msg.angular.z=.4
            self.vel_msg.linear.x=0.01
    def Frame_Slicer(self,image_stream):
        top_slice = Line_Tracker(.6,.7,image_stream,"top centroid")
        middle_slice = Line_Tracker(.8,.9,image_stream,"middle centroid")
        bottom_slice = Line_Tracker(.9,1,image_stream,'bottom centroid')
        self.top_centroid_x, self.top_centroid_y, self.current_frame, top_missing = top_slice.centroid_posistion()
        if not top_missing:
            global found_line;found_line=True
        self.middle_centroid_x,self.middle_centroid_y, trashThis, junk = middle_slice.centroid_posistion()
        self.bottom_centroid_x,self.bottom_centroid_y, whyWasteMemory, self.missing_lower = bottom_slice.centroid_posistion()
        cv2.circle(self.current_frame,(int(self.top_centroid_x), int(self.top_centroid_y)), 5,(245,10,10),-1)
        cv2.circle(self.current_frame,(int(self.middle_centroid_x), int(self.middle_centroid_y)), 5,(10,240,10),-1)
        cv2.circle(self.current_frame,(int(self.bottom_centroid_x), int(self.bottom_centroid_y)), 5,(10,10,255),-1)
        #cv2.imshow('Centroids',self.current_frame)
        if not self.missing_lower:
            global on_line; on_line = True
        #cv2.waitKey(1)
    def Angle(self):
        control_signals=[self.short_angle_controller.steer(self.path_angle[1]), self.mid_angle_controller.steer(self.path_angle[2]),self.long_angle_controller.steer(self.path_angle[3])]
        output=np.deg2rad(np.average(control_signals,weights=[3,1,5]))
        #print(control_signals, output)
        return output
    def Flag_Setter(self,status):
        #print(status)
        if True:
            global stop_sign; stop_sign = True
            print("seen stop sign")
    def Stopper(self,Scan):
        #print("Hi")
        if stop_sign == True:    
            ranges = np.array(Scan.ranges)
            ranges[np.where(ranges == np.inf)] = LIDAR_INF
            ranges[np.where(ranges == 0)] = LIDAR_INF
            print(np.average(ranges[285:290]))
            #print("Stop Checker")
            if np.average(ranges[285:290])<0.6 and self.not_stopped:
                print("stopping")
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.not_stopped = False
                self.pub.publish(self.vel_msg)
                self.pub.unregister()
                time.sleep(3)
                self.pub = rospy.Publisher('/cmd_vel_camera',Twist, queue_size=10)#Publisher Node
                print("Not_Stopping")
    def Shutdown(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z= 0
        self.pub.publish(self.vel_msg)
    
class Line_Tracker:
    # Kernels for blurring and edge detection 
   kernel_edge_3 = np.array([[-1,-1,-1],
                           [-1,8,-1],
                           [-1,-1,-1]])
   kernel_edge_5 = np.array([[-1,-1,-1,-1,-1],
                              [-1,0,0,0,-1],
                              [-1,0,16,0,-1],
                              [-1,0,0,0,-1],
                              [-1,-1,-1,-1,-1]])
   kernel_blur = np.array([[1/9,1/9,1/9],
                            [1/9,1/9,1/9],
                            [1/9,1/9,1/9]])
   blue_lower = np.array([15,30,60])
   blue_upper = np.array([50,170,255])

   def __init__(self,upper_bound_decimal, lower_bound_decimal,image_stream,output_name):
       self.bridge = CvBridge()#CvBridge Function
       self.upper_bound_decimal = upper_bound_decimal
       self.lower_bound_decimal = lower_bound_decimal
       self.image_stream = image_stream
       self.output_name = output_name
       self.current_frame = self.bridge.imgmsg_to_cv2(self.image_stream) #Imports Current Image
       #self.current_frame = cv2.normalize(self.current_frame, None,30,170)
       #cv2.imshow("Original",self.current_frame)
   def centroid_locate(self):
       current_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)#Converts to HSV
       self.height, self.width, channels = current_frame.shape#Gets dimensions of the image
       crop = current_frame[int(self.height*self.upper_bound_decimal):int(self.height*self.lower_bound_decimal),int(0.*self.width):int(0.96*self.width)]#Crops images down
    #    cv2.imshow(self.output_name+self.output_name,crop)
    #    cv2.waitKey(1)
       mask=cv2.inRange(crop,self.blue_lower,self.blue_upper)#Generates mask to exclude non-blue regions
    #    edge_img=cv2.bitwise_and(crop,crop,mask=mask)#Maskes image
    #    edge_img=cv2.filter2D(edge_img,ddepth=-1,kernel = self.kernel_edge_5)#Uses 5x5 edge detection filter
    #    ret,edge_threshold=cv2.threshold(edge_img,30,255,cv2.THRESH_BINARY)
       m = cv2.moments(mask, False)
       try:
            self.cx, self.cy = m['m10']/m['m00'], m['m01']/m['m00']
       except ZeroDivisionError:
            self.cx, self.cy = self.width/2, self.height/2
       #print("Mask"+self.output_name+":")
       #print(self.cx)
    #    cv2.circle(edge_threshold,(int(self.cx), int(self.cy)), 10,(120,120,120),-1)
       #cv2.imshow(self.output_name,mask)
       if np.sum(mask)<500:
           self.missingLower = True
       else:
           self.missingLower = False
       #cv2.waitKey(1)
   def centroid_posistion(self):
      self.centroid_locate()
      absolute_pos_y= self.cy+(self.height*self.lower_bound_decimal)
      absolute_pos_x=self.cx+(self.width*0.)
      return absolute_pos_x, absolute_pos_y,self.current_frame, self.missingLower
   
class traffic_sign:
    def __init__(self):
        # self.sub = rospy.Subscriber('/stop_sign',Bool,self.Flag_Setter)
        self.sub = rospy.Subscriber('/scan',LaserScan,self.Stopper)
        self.pub = rospy.Publisher('/cmd_vel_camera',Twist, queue_size=10)#Publisher Node
        self.not_stopped = True
        self.vel_msg = Twist()
    # def Flag_Setter(self,status):
    #     if status == True:
    #         global stop_sign; stop_sign = True
    def Stopper(self,Scan):
        #print("Hi")
        if stop_sign == True:    
            ranges = np.array(Scan.ranges)
            ranges[np.where(ranges == np.inf)] = LIDAR_INF
            ranges[np.where(ranges == 0)] = LIDAR_INF
            print(np.average(ranges[285:290]))
            #print("Stop Checker")
            if np.average(ranges[285:290])<0.6 and self.not_stopped:
                print("stopping")
                
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.not_stopped = False
                self.pub.publish(self.vel_msg)
                time.sleep(3)
                print("Not_Stopping")
if __name__ == '__main__':
    try:
        lf=Turtlebot_Movement()
        ts=traffic_sign()
        rospy.spin()
        rospy.on_shutdown(lf.Shutdown) 
    except rospy.ROSInterruptException:
        pass
