#!/usr/bin/env python3
import rospy

from turtlebot_autonav.msg import Pulse

class TFDrop:
    def __init__(self):            
        rospy.init_node('tfDrop', anonymous=True)
        
        ''' Publishers '''        
        self.pub_drop = rospy.Publisher('/dropTf', Pulse, queue_size=5)     
        self.sub_alive = rospy.Subscriber('/pulse', Pulse, self.publish)
                       
    def publish(self, data):                
        self.pub_drop.publish(Pulse())            
        

if __name__ == '__main__':
    try:
        tfDropNode = TFDrop()        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
