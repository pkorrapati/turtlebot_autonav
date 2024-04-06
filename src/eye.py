#!/usr/bin/env python3

import cv2
import imagezmq
import os
import rospy

rate = 50 # fps

class Eye:
        def __init__(self):
            rospy.init_node('eye', anonymous=True)

            self.rate = rospy.Rate(rate)

            rMasterURI = os.environ['ROS_MASTER_URI']
            self.vs = cv2.VideoCapture(0)
            self.awake = False

            if rMasterURI:
                self.hostIP = os.environ['ROS_MASTER_URI'].split(':')
                self.hostIP = self.hostIP[1] if len(self.hostIP) > 1 else ''
            
            if self.hostIP:
                self.sender = imagezmq.ImageSender(connect_to='tcp:' + self.hostIP + ':5555', REQ_REP = False)                
                self.awake = True
           
        def stream(self):
              while not rospy.is_shutdown() and self.awake:
                frame = self.vs.read()[1]                
                self.sender.send_image('turtlebot', frame)
                self.rate.sleep()

if __name__ == '__main__':
    try:        
        eye = Eye()
        eye.stream()

    except rospy.ROSInterruptException:
        pass