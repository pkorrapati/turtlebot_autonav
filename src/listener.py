#!/usr/bin/env python3
import rospy #ROS library for python
from std_msgs.msg import Bool
class eye_oh:
    def __init__(self):
        rospy.init_node('line_recognition=.py', anonymous=True)
        self.rate = rospy.Rate(500)#Sets refresh rate
        self.pub = rospy.Publisher('/state',Bool, queue_size=10)#Publisher Node
        self.pub.publish(True)
    def state_machine(self):
        while not rospy.is_shutdown():
            print("Press C to enter Computer vision mode")
            state = input().lower().strip()
            if state =='c':
                self.pub.publish(False)
if __name__ == '__main__':
    try:
        io=eye_oh()
        io.state_machine()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass