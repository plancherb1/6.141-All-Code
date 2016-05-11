#!/usr/bin/python
#
import rospy
import math
import numpy as np
from geometry_msgs.msg import Point # float 64 x,y,z

class ControllerNode2:
    def __init__(self):
        # subscribe to the visual and obstacle goals
        rospy.Subscriber("racecar/localization/targetPoint", Point, self.TargetUpdate)
        rospy.Subscriber("racecar/visual_goal", Point, self.VisualUpdate)

        # advertise that we'll publish the final point for steering
	self.dir_pub = rospy.Publisher("racecar/controls/direction", Point, queue_size = 0)

	# global variables to store values and timestamps
        self.target_goal = None #default to none
	self.target_goal_time = None #timestamp to see if still valid
	self.visual_goal = None #default to none
	self.visual_goal_time = None #timestamp to see if still valid
	self.time_delta = rospy.Duration.from_sec(0.125) #time where our visual_goal is still valid for planning

	# default goal
	self.default_goal = Point(5,10,0) #straight ahead

    # Callback funcs to update value and send new goal
    def TargetUpdate(self,target_goal):
	self.target_goal = target_goal
	self.target_goal_time = rospy.get_rostime()
	self.find_local_goal()

    def VisualUpdate(self,visual_goal):
	self.visual_goal = visual_goal
	self.visual_goal_time = rospy.get_rostime()
	self.find_local_goal()

    # Picks the goal based on the flags
    def pick_goal(self,vf,tf,vg,tg,dg):
	if (vf and tf):
	    return vg
	    # if visual goal is in the 180deg arc of hlg goal use it
	    theta = abs(vg[2]-tg[2]) % (math.pi*2)
	    if (theta <= math.pi):
	        return vg
	    # else high level goal
	    else:
	        return tg
        elif (vf):
	    return vg
        elif (tf): 
	    return tg
	else:
	    return dg

    # Publish the goal direction
    def find_local_goal(self):
	target_flag = (not (self.target_goal == None)) and (abs(self.target_goal_time - rospy.get_rostime()) < self.time_delta)
	visual_flag = (not (self.visual_goal == None)) and (abs(self.visual_goal_time - rospy.get_rostime()) < self.time_delta)
        # then get the directional goal_point
        directional_goal = self.pick_goal(visual_flag,target_flag,self.visual_goal,self.target_goal,self.default_goal)
	self.dir_pub.publish(directional_goal)

if __name__ == '__main__':
    rospy.init_node("ControllerNode2")
    node = ControllerNode2()
    rospy.spin()
