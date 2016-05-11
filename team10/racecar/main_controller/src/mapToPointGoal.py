#!/usr/bin/python
#
import rospy
from geometry_msgs.msg import Point # float 64 x,y,z

class MapToPointGoalNode:
    def __init__(self):
	# subscribe to the gloabl map with us positioned
        rospy.Subscriber("/racecar/localization/targetPoint", Point, self.callback)

 	# advertise that we'll publish on the high level goal topic
        self.pub = rospy.Publisher("racecar/high_level_goal", Point, queue_size=1)

    # Parse map and return a global goal point (can just be unit vector since we
    # use it in a dot product and really just the direction is important
    def callback(self, point):
	# currently we are just being sent a point so publish it!
        self.pub.publish(Point(point.x,point.y,point.z))

if __name__ == '__main__':
    rospy.init_node("MapToPointGoalNode")
    node = MapToPointGoalNode()
    rospy.spin()
