#!/usr/bin/env python3
import rospy #ROS library for python
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
class eye_oh:
    def __init__(self):
        rospy.init_node('line_recognition=.py', anonymous=True)
        self.rate = rospy.Rate(500)#Sets refresh rate
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)#Publisher Node
        self.obstacle = rospy.Subscriber('/cmd_vel_obstacle',Twist,self.Obs)
        self.camera = rospy.Subscriber('/cmd_vel_camera',Twist,self.Cam)
        self.state = rospy.Subscriber('/state',Bool,self.state_change)
        self.cam_vel=Twist()
        self.obs_vel=Twist() 
    def Cam(self,vel):
        self.cam_vel=vel
    def Obs(self,vel):
        self.obs_vel=vel
    def state_change(self,state):
        self.state=state
    def state_machine(self):
        while not rospy.is_shutdown():
            if self.state==True:
                cmd_vel= self.obs_vel
                print("In Obstacle Avoidance Mode")
                print(self.state)
            else:
                cmd_vel=self.cam_vel
                print("In CV mode!")
            self.pub.publish(cmd_vel)
if __name__ == '__main__':
    try:
        io=eye_oh()
        io.state_machine()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass