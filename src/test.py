from math import radians, atan2, pi, degrees
import numpy as np

crashDist = 0.3
tbWidth = 0.2

def extractRanges(ranges, center, Mins, Plus):
    if (center - Mins) < 0 and (center + Plus) > 0 :
        return np.hstack((ranges[(center - Mins):], ranges[:(center + Plus)]))
    else:
        return np.array(ranges[(center - Mins):(center + Plus)])

class VisualCortex:
    def __init__(self):        
        self.fAngle = radians(0)
        self.rAngle = self.fAngle - radians(90)
        self.lAngle = self.fAngle + radians(90)

        # Range of front crash angles
        # crash is largest angle of triangle at smallest distance
        crashAngle = atan2(tbWidth, (2*crashDist))

        self.fPlus = radians(crashAngle)
        self.fMins = radians(crashAngle)

    def analyze(self, data):                
        # self.pub_motion.publish(self.vel)     
        self.pulRate = data.rate   

    def visualize(self, data):
        pass
    
    def echo(self, data):
        incAngle = data.angle_increment
        minAngle = data.angle_min
                
        ranges = np.array(data.ranges)
        count = len(ranges)
        
        # Clean up inf in data
        ranges[np.where(ranges == np.inf)] = 10.0

        # Indices of Left End and Right End of Region of Interest
        # use these only with data.ranges

        # TODO : During calibration, we might get offsets for front, left and right cases
        # update code to handle + and - cases of each angles. Especially front
        fIndx = round((self.fAngle - minAngle)/incAngle) # Index of heading
        lIndx = round((self.lAngle - minAngle)/incAngle) # Index of left        
        rIndx = round((self.rAngle - minAngle)/incAngle)
        # rIndx = round((self.rAngle - minAngle)/incAngle) if self.rAngle > minAngle else (round(((2*pi + self.rAngle) - minAngle)/incAngle) - count)

        # print(fIndx)
        # print(ranges[fIndx])

        # print(lIndx)
        # print(ranges[lIndx])

        # print(rIndx)
        # print(ranges[rIndx])
        
        # # Front Half Scan Region
        self.frontHalfRanges = np.hstack((ranges[rIndx:], ranges[:lIndx+1]))
        frontHalfCount = len(self.frontHalfRanges) #lIndx - rIndx

                
        self.frontHalfAngles = np.linspace(self.rAngle, self.lAngle, num=frontHalfCount)

        print(self.frontHalfRanges)
        print(frontHalfCount)
        print(np.degrees(self.frontHalfAngles))

        # # Generate continuous angles from Right to Left
        # if len(self.frontHalfAngles) != frontHalfCount:            
        #     self.frontHalfAngles = np.linspace(self.rAngle, self.lAngle, num=frontHalfCount)
        
        # # Front crash region
        # self.fPlusIndx = round(self.fPlus/incAngle)
        # self.fMinsIndx = round(self.fMins/incAngle)

        # # front scan
        # self.crashScan = extractRanges(ranges, fIndx, self.fMinsIndx, self.fPlusIndx)
        # self.crashRegion = np.linspace(self.fAngle - self.fMins, self.fAngle + self.fPlus, num=len(self.crashScan))

        # self.newEcho = True

class x:
    def __init__(self):
        pass

a = x()
a.ranges = range(0,360)
a.angle_min = radians(0)
a.angle_increment = radians(1)

vNode = VisualCortex()
vNode.echo(a)

