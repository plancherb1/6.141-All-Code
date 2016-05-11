#!/usr/bin/python
#
import rospy
from math import cos
from math import sin
from geometry_msgs.msg import Point # float 64 x,y,z
from std_msgs.msg import Float64MultiArray

class DataToVisualGoalNode:
    def __init__(self):
	# subscribe to the OpenCV output
        rospy.Subscriber("/racecar/image/data", Float64MultiArray, self.callback)

 	# advertise that we'll publish on the high level goal topic
        self.pub = rospy.Publisher("racecar/visual_goal", Point, queue_size=1)

    # Parse data and extract the nearest goal
    def callback(self, data):
	# data is just theta so x,y is bs
	theta = data.data[0]
	if not (theta == -1):
	    self.pub.publish(Point(0,0,theta))

if __name__ == '__main__':
    rospy.init_node("DataToVisualGoalNode")
    node = DataToVisualGoalNode()
    rospy.spin()
