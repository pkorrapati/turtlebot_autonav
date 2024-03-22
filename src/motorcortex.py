#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist

class MotorCortex:
    def __init__(self):
        # Creates a unique node 'motor_cortex' by using anonymous=True
        rospy.init_node('motor_cortex', anonymous=True)

        # Publisher for topic '/cmd_vel'.
        self.motion_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
        x = MotorCortex()
        x.traverseCircle()
    except rospy.ROSInterruptException:
        pass
