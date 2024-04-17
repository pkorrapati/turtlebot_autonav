#!/usr/bin/env python3
import rospy #ROS library for python
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
class helmsman:
    def __init__(self):
        print("hi")
        self.current_state = False
        rospy.init_node('helm', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)#Publisher Node
        self.obstacle = rospy.Subscriber('/cmd_vel_obstacle',Twist,self.Obs)
        self.camera = rospy.Subscriber('/cmd_vel_camera',Twist,self.Cam)
        self.state = rospy.Subscriber('/helm/line',Bool,self.state_change)
        self.cam_vel=Twist()
        self.obs_vel=Twist() 
    def Cam(self,vel):
        self.cam_vel=vel
        self.state_machine()
    def Obs(self,vel):
        self.obs_vel=vel
        self.state_machine()
    def state_change(self,state):
        self.current_state=state
        self.state_machine()
    def state_machine(self):
        print(self.state)
        if self.current_state==False:
            cmd_vel= self.obs_vel
            print("In Obstacle Avoidance Mode")
            print(self.state)
        else:
            cmd_vel=self.cam_vel
            print("In CV mode!")
        self.pub.publish(cmd_vel)
if __name__ == '__main__':
    try:
        helm=helmsman()
        rospy.spin()
        pass
    except rospy.ROSInterruptException:
        pass
