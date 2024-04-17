#!/usr/bin/env python3
import rospy
import numpy as np

from turtlebot_autonav.msg import Pulse, MotionCmd
from geometry_msgs.msg import Twist, Quaternion
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

import tf2_ros
from math import pi, radians, cos, sin, acos
import matplotlib.pyplot as plt

# Remaps (-pi/2, pi/2) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

def getRotation(q):    
    return np.array([[q.w**2 + q.x**2 - q.y**2 -q.z**2, 2 * q.x * q.y - 2 * q.w * q.z],
                     [   2 * q.x * q.y + 2 * q.w * q.z, q.w**2 - q.x**2 + q.y**2 -q.z**2 ]])    

def getTransform(x,y, q):
    R = getRotation(q)    
    T = np.eye(3)

    T[:2, :2] = R
    T[:2, 2] = [x, y]

    return T

def matInv(Tr):
    R_inv = Tr[:2, :2].T
    t_inv = np.dot(-1, np.dot(R_inv, Tr[:2, 2]))

    Tr_inv = np.eye(3)
    Tr_inv[:2, :2] = R_inv
    Tr_inv[:2, 2] = t_inv

    return Tr_inv

class PrefrontalCrotex:
    def __init__(self):        
        self.isAlive = False
        rospy.init_node('PrefrontalCrotex', anonymous=True)


        # Subscribers
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.flow)
        self.sub_stop = rospy.Subscriber('/dropTf', Pulse, self.stopDetected)
        
        # Publishers
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)        

        # Listen to tf frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_subber = tf2_ros.TransformListener(self.tfBuffer)

        q = Quaternion()
        q.z = 1

        self.iniPose = getTransform(0,0,q)
        self.pose = getTransform(0,0,q)
        self.dest = getTransform(0,0,q)

        self.reachedDestination = False
        self.hasDesitination = False

        poseSet = False

        while not poseSet:
            try:
                transform = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time()).transform #base_footprint
                self.iniPose = getTransform(transform.translation.x, transform.translation.y, transform.rotation)
                self.pose = self.iniPose
                
                poseSet = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

        self.isAlive = True

    def flow(self, data):        
        if self.isAlive:        
            transform = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time()).transform #base_footprint        
            pose = getTransform(transform.translation.x, transform.translation.y, transform.rotation)        
            self.pose = np.dot(matInv(self.iniPose), pose)

            if self.hasDesitination:   
                differ = np.dot(matInv(self.pose), self.dest)                
                print(differ)
                # print(np.sqrt(np.linalg.norm(differ[:2])))

                if np.sqrt(np.linalg.norm(differ[:2])) < 0.2:
                    self.reachedDestination = True
                    print("reached")

            # print(self.pose)

    def stopDetected(self, data):                
        dist = [0.5, 0, 1]        
        
        if self.isAlive and not self.hasDesitination:                    
            self.dest = np.dot(self.pose, dist)
            self.hasDesitination = True

            print("Destination Set")

    # def traverseCircle(self):
    #     """ Move in a circle. """
        
    #     # Get the input from the user.        
    #     r = rospy.get_param('~r')
    #     w = rospy.get_param('~w')

    #     vel_msg = Twist()
        
    #     rotPeriod = 2*pi/w # Time period to rotate

    #     # wait for robot to spawn
    #     while not self.isAlive:
    #         pass

    #     # Send a stop signal
    #     vel_msg.linear.x = 0
    #     vel_msg.linear.y = 0
    #     vel_msg.linear.z = 0

    #     # Angular velocity in the z-axis.
    #     vel_msg.angular.x = 0
    #     vel_msg.angular.y = 0
    #     vel_msg.angular.z = 0

    #     # Publishing our vel_msg
    #     self.velocity_publisher.publish(vel_msg)
    #     self.rate.sleep()

    #     # Using inbuilt function get_time that listens to /clock topic               
    #     t_start = rospy.get_time()

    #     # print(self.tfBuffer.all_frames_as_yaml())

    #     X =[]
    #     Y =[]

    #     while rospy.get_time() <= t_start + rotPeriod:
    #         try:                
    #             trans = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time()) #base_footprint
    #             X.extend([trans.transform.translation.x])
    #             Y.extend([trans.transform.translation.y])
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             continue

    #         # Linear velocity in the x-axis.
    #         vel_msg.linear.x = w * r
    #         vel_msg.linear.y = 0
    #         vel_msg.linear.z = 0

    #         # Angular velocity in the z-axis.
    #         vel_msg.angular.x = 0
    #         vel_msg.angular.y = 0
    #         vel_msg.angular.z = w

    #         # Publishing our vel_msg
    #         self.velocity_publisher.publish(vel_msg)

    #         # Publish at the desired rate.
    #         self.rate.sleep()

    #     # Stop motion
    #     vel_msg.linear.x = 0
    #     vel_msg.linear.y = 0
    #     vel_msg.linear.z = 0

    #     vel_msg.angular.x = 0
    #     vel_msg.angular.y = 0
    #     vel_msg.angular.z = 0
    #     self.velocity_publisher.publish(vel_msg)


        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    try:
        pfCortex = PrefrontalCrotex()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
