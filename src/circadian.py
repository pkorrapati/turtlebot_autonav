import rospy
from final_prj.msg import Pulse

pulseRate = 100

class Circadian:
    def __init__(self, beatsPerSecond=100):
        # Creates a unique node 'motor_cortex' by using anonymous=True
        rospy.init_node('circadian', anonymous=True)

        # Publisher for topic '/pulse'
        self.rate_publisher = rospy.Publisher('/pulse', Pulse, queue_size=10)

        self.rate = rospy.Rate(beatsPerSecond)
        self.isAlive = False     
        
        self.pulse = Pulse()
        self.pulse.rate = beatsPerSecond

    def stayAlive(self):
        if not self.isAlive:
            self.isAlive = True

        while not rospy.is_shutdown():
            self.rate_publisher.publish(self.pulse)
            self.rate.sleep()  

        self.isAlive = False      

if __name__ == '__main__':
    try:
        x = Circadian(pulseRate)
        x.stayAlive()
    except rospy.ROSInterruptException:
        pass