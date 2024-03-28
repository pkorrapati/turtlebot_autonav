#!/usr/bin/env python3
import rospy

from turtlebot_autonav.msg import Pulse, MotionCmd
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

import tf2_ros
from math import pi, radians
import matplotlib.pyplot as plt

# Remaps (-pi/2, pi/2) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

class TurtleBot:

    def __init__(self):
        # Creates a node 'turtlebot_circle_driver'. Using anonymous=True makes it unique.
        rospy.init_node('turtlebot_circle_driver', anonymous=True)

        # Reset turtlesim by calling the reset service
        # Not necessary when running from for launch files
        rospy.wait_for_service('/gazebo/reset_world')
        resetTurtle = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        # Listen to tf frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_subber = tf2_ros.TransformListener(self.tfBuffer)

        self.rate = rospy.Rate(100)   
        self.isAlive = False     

    def update_pose(self, data):
        if not self.isAlive:
            self.isAlive = True

    def traverseCircle(self):
        """Move in a circle."""
        
        # Get the input from the user.        
        r = rospy.get_param('~r')
        w = rospy.get_param('~w')

        vel_msg = Twist()
        
        rotPeriod = 2*pi/w # Time period to rotate

        # wait for robot to spawn
        while not self.isAlive:
            pass

        # Send a stop signal
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

        # Using inbuilt function get_time that listens to /clock topic               
        t_start = rospy.get_time()

        # print(self.tfBuffer.all_frames_as_yaml())

        X =[]
        Y =[]

        while rospy.get_time() <= t_start + rotPeriod:
            try:                
                trans = self.tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time()) #base_footprint
                X.extend([trans.transform.translation.x])
                Y.extend([trans.transform.translation.y])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            # Linear velocity in the x-axis.
            vel_msg.linear.x = w * r
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stop motion
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        plt.plot(X,Y)
        plt.title('X-Y location of base_footprint')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.show()

        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.traverseCircle()
    except rospy.ROSInterruptException:
        pass
