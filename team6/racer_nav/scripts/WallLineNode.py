#!/usr/bin/python
#
import rospy
import math
from scipy import stats
from sensor_msgs.msg import LaserScan
from racecar_nav.msg import WallLines
# Need to install on the VM and Car sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose

class WallLineNode:
    def __init__(self):
        
        # subscribe to the scan topic; comment out which one you want to use
        """ On Baymax """
        #rospy.Subscriber("scan", LaserScan, self.WallLineCallback)
        """ On Simulator """
        rospy.Subscriber("/racecar/laser/scan", LaserScan, self.WallLineCallback)
        
        # advertise that we'll publish on the WallLine topic
        self.WallLinePub = rospy.Publisher("WallLine", WallLines, queue_size = 10)

    def FindLongestLine(self,xs,ys,BubbleSize):
        # guess that the wall occurs in a range close to us and iterate up until we
        # have r < RTHRESHOLD or used the whole space
	RTHRESHOLD = 0.05
        rangesize = 5
        middleIndex = BubbleSize/2
        slope = 0
        intercept = 0
        while True:
	    slope, intercept, r_value, p_value, std_err = stats.linregress(xs[middleIndex - rangesize:middleIndex + rangesize],ys[middleIndex - rangesize:middleIndex + rangesize])
            if abs(r_value) > RTHRESHOLD:
                if rangesize * 3 < BubbleSize:
                    rangesize = rangesize * 3
                else:
                    break
	    else:
		rangesize = rangesize / 3
		break
        P1 = (xs[middleIndex - rangesize],ys[middleIndex - rangesize])
        P2 = (xs[middleIndex + rangesize],ys[middleIndex + rangesize])
        return P1+P2
        
    def WallLineCallback(self, LaserScan_msg):
        # Laser index 0:1080, using 0:405 for left bubble and 675: for right detection bubble
        RightBubble = LaserScan_msg.ranges[0:405]
        LeftBubble = LaserScan_msg.ranges[675:]
        BubbleSize = len(LeftBubble)

        # some constants to work with
        ZEROINDEXINDEG = -45
        INDEXTODEG = 0.25
        DEGTORAD = math.pi/180
        RTHRESHOLD = 0.8

        leftXs = []
        leftYs = []
        rightXs = []
        rightYs = []
        # transform from polar to rectangular
        for i in range(BubbleSize):
	    k = i + 675
            theta = (ZEROINDEXINDEG+k*INDEXTODEG)*DEGTORAD
            r = LeftBubble[i]
            if r > 9.5:
		leftXs += [0]
            	leftYs += [0]
	    else:
            	leftXs += [r*math.cos(theta)]
            	leftYs += [r*math.sin(theta)]
        for j in range(BubbleSize):
            k = j + 0
            theta = (ZEROINDEXINDEG+k*INDEXTODEG)*DEGTORAD
            r = RightBubble[j]
	    if r > 9.5:
		rightXs += [0]
            	rightYs += [0]
	    else:
	        rightXs += [r*math.cos(theta)]
            	rightYs += [r*math.sin(theta)]

        # find the linear equations describing the walls
        leftPoints = self.FindLongestLine(leftXs,leftYs,BubbleSize)
        rightPoints = self.FindLongestLine(rightXs,rightYs,BubbleSize)

        #create the object to publish the two sets of points that define the lines
        returnVal = WallLines()
        returnVal.leftPoints = leftPoints
        returnVal.rightPoints = rightPoints	

        # publish the object
        self.WallLinePub.publish(returnVal)     

if __name__ == '__main__':
    rospy.init_node("WallLineNode")
    node = WallLineNode()
    rospy.spin()
