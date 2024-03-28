#!/usr/bin/env python3

#rostopic pub /motion_cmd turtlebot_autonav/MotionCmd "lin: 1"
import rospy

from turtlebot_autonav.msg import Pulse, MotionCmd
from geometry_msgs.msg import Twist

# Move to a YAML file if possible
max_lin_vel = 0.2
max_rot_vel = 0.2

max_lin_acc = 0.2
max_rot_acc = 0.2

def limit(value, limitL, limitU):
    if value > limitU:
        return limitU
    elif value < limitL:
        return limitL
    
    return value

class MotorCortex:
    def __init__(self):        
        rospy.init_node('motor_cortex', anonymous=True)

        self.vel = Twist()

        # Subscribers
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.mobilize)
        self.sub_motion_cmd = rospy.Subscriber('/motion_cmd', MotionCmd, self.set_motion)
    
        # Publishers
        self.pub_motion = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def set_motion(self, data):
        pul = self.pulRate if self.pulRate != 0 else 100

        if not data.brake:
            self.vel.linear.x = limit(self.vel.linear.x + (data.lin * max_lin_acc / pul), -max_lin_vel, max_lin_vel)
            self.vel.angular.z = limit(self.vel.angular.z + (data.rot * max_rot_acc / pul), -max_rot_vel, max_rot_vel)
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0

    def mobilize(self, data):                
        self.pub_motion.publish(self.vel)     
        self.pulRate = data.rate   

if __name__ == '__main__':
    try:
        mNode = MotorCortex()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
