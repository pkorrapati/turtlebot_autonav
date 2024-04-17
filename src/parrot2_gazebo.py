#!/usr/bin/env python3

##Stop Sign Identifer used:https://github.com/maurehur/Stop-Sign-detection_OpenCV/tree/master
import rospy #ROS library for python
from sensor_msgs.msg import Image, LaserScan#ROS message format for images
import cv2#OpenCV library
from cv_bridge import CvBridge#Bridge to go between ros and opencv
import numpy as np
from std_msgs.msg import Bool
# Load haar cascade xml file
stop_sign_cascade = cv2.CascadeClassifier('/turtle_ws/src/turtlebot_autonav/src/stop_sign_classifier_2.xml')

class Sqawk:
    def __init__(self):
        rospy.init_node('navigator.py', anonymous=True)
        self.pub = rospy.Publisher('/parrot/stop',Bool, queue_size=10)#Publisher Node
        self.sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.Stop_Check)
        self.bridge = CvBridge()#CvBridge Function
    def Stop_Check(self,Image):
        image = self.bridge.imgmsg_to_cv2(Image)
        image_np = np.array(image)
        # image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
	    # Apply Gaussian filter
        gray_filered = cv2.GaussianBlur(gray, (5, 5), 0)
        cv2.imshow("hi",gray_filered)
        # Detection
        stop_signs = stop_sign_cascade.detectMultiScale(gray_filered, scaleFactor=1.05, minNeighbors=5, minSize=(5, 5))
        if len(stop_signs)>0:
              self.pub.publish(True)
        for (x,y,w,h) in stop_signs:
            cv2.rectangle(image_np, (x, y), (x+w, y+h), (255, 255, 0), 2)

        # cv2.namedWindow("screenshot", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('screenshot', 640, 480)
        # cv2.imshow('screenshot',image_np)

        #cv2.waitKey(1)
if __name__ == '__main__':
    try:
        polly=Sqawk()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
