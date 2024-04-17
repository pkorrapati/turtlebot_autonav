#!/usr/bin/env python3

import rospy

from turtlebot_autonav.msg import Pulse

pulseRate = 1000

class Circadian:
    def __init__(self, beatsPerSecond=1000):
        # Creates a unique node 'motor_cortex' by using anonymous=True
        rospy.init_node('circadian', anonymous=True)

        self.rate = rospy.Rate(beatsPerSecond)
        self.isAlive = False     
        
        self.pulse = Pulse()
        self.pulse.rate = beatsPerSecond

        # Publishers
        self.pub_rate = rospy.Publisher('/pulse', Pulse, queue_size=10)        

    def stay_alive(self):
        if not self.isAlive:
            self.isAlive = True

        while not rospy.is_shutdown():
            self.pub_rate.publish(self.pulse)
            self.rate.sleep()  

        self.isAlive = False      

if __name__ == '__main__':
    try:
        #pul = rospy.get_param('~pulseRate', default=pulseRate)
        cNode = Circadian(1000)
        print()
        cNode.stay_alive()

    except rospy.ROSInterruptException:
        pass