#!/usr/bin/python
#
import rospy
import math
import numpy as np
from time import time
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point # float 64 x,y,z
from main_controller.msg import OccupancyGrid2

class ControllerNode:
    def __init__(self):
        # subscribe to the expanded occupancy grid, visual, emergency, and high level goal
        rospy.Subscriber("racecar/occupancy_grid_expanded", OccupancyGrid2, self.CalcGoal)
        rospy.Subscriber("racecar/emergency_stop", Bool, self.Emergency)
        rospy.Subscriber("racecar/high_level_goal", Point, self.HighLevelUpdate)
        rospy.Subscriber("racecar/visual_goal", Point, self.VisualUpdate)

        # advertise that we'll publish the local goal and the grid
	self.grid_pub = rospy.Publisher("racecar/occupancy_grid_final", OccupancyGrid2, queue_size = 1)

	# global variables to store values and timestamps
	self.curr_occupancy_grid = None #default to none
        self.high_level_goal = None #default to none
	self.visual_goal = None #default to none
	self.visual_goal_time = None #timestamp to see if still valid
	self.grid_time = None #timestamp to know when we got a new grid
	self.emergency_time_delta = 0.5 #time where our grid is still valid for re-plan
	self.visual_time_delta = 0.5 #time where our visual_goal is still valid for planning

	# default goal
	self.default_goal = Point(5,10,0) #straight ahead

    # math helper funcs
    def dot2DTuple(self,a,b):
	return (b[0]*a[0] + b[1]*a[1])
    def mag2DTuple(self,a):
	return math.sqrt(a[0]**2 + a[1]**2)
    def diff2DPointsToTuple(self,a,b):
	return (b.x-a.x,b.y-a.y)
    def angleFrom2DTuple(self,a,b):
	# then use a dot b = mag(a)mag(b)costheta
	mag = self.mag2DTuple(a) * self.mag2DTuple(b)
	dot = self.dot2DTuple(a,b)
	try:
	    return math.acos(dot/mag)
        except:
	    return 0
    # convert an index into an (x,y)
    def pointFromIndex(self,i,length,meters,theta):
	scale = meters/length
    	curr_y = i % int(length) * scale
	curr_x = i / int(length) * scale
	return Point(curr_x,curr_y,theta)

    # Callback funcs
    def HighLevelUpdate(self,high_level_goal):
	self.high_level_goal = high_level_goal

    def VisualUpdate(self,visual_goal):
	self.visual_goal = visual_goal
	self.visual_goal_time = rospy.get_rostime()

    # Recompute on emergency if possible
    def Emergency(self,flag):
	# if we get back a positive flag that we had an emergency
	if flag:
	    # see if the grid is still valid
	    if (not (self.visual_goal == None)) and (self.grid_time - rospy.get_rostime() < self.emergency_time_delta):
	        # if so recalculate path now
		self.CalcGoal(self.curr_occupancy_grid)
            # else just wait stopped for a new grid

    # Computes the local goal and adds it to the OccupancyGrid2 and publishes
    def CalcGoal(self,grid):
	# timestamp
	self.grid_time = rospy.get_rostime()
        # get the start
	start = grid.start
        # see if we need to combine visual and high level goals
	high_flag = (not (self.high_level_goal == None))
	visual_flag = (not (self.visual_goal == None)) and (self.visual_goal_time - rospy.get_rostime() < self.visual_time_delta)
        # then get the directional goal_point
        directional_goal = self.pick_goal(visual_flag,high_flag,self.visual_goal,self.high_level_goal,self.default_goal,start)
        # parse the grid and find the local goal based on the directional_goal
	grid.goal = self.find_local_goal(grid,directional_goal,self.default_goal)
	# then publish the grid and the goal
	self.grid_pub.publish(grid)

    # Picks the goal based on the flags
    def pick_goal(self,vf,hf,vg,hg,dg,s):
	if (vf and hf):
	    # if visual goal is in the 180deg arc of hlg goal use it
	    # first get delta vector to the vg and hg from start
	    s_vg = self.diff2DPointsToTuple(s,vg)
	    s_hg = self.diff2DPointsToTuple(s,hg)
	    theta = self.angleFrom2DTuple(s_vg,s_hg)
	    if (theta <= math.pi):
	        return vg
	    # else high level goal
	    else:
	        return hlg
        elif (vf):
	    return vg
        elif (hf): 
	    return hg
	else:
	    return dg

    # Find the appropriate local goal
    def find_local_goal(self,grid,directional_goal,default):
	# find longest projection of a vector from start to an unblocked square
	# in the direction of the directional_goal
	start = grid.start
	goal_vect = self.diff2DPointsToTuple(start,directional_goal)
	# check all points on grid for best local goal (time N)
	w = int(grid.width)
	l = int(grid.length)
	best_goal = default
	best_score = 0
	for i in range(w*l):
	    # if a vaild space
	    if not grid.grid.data[i]:
		# calc the x,y point
	 	curr_point = self.pointFromIndex(i,grid.length,grid.meters,directional_goal.z)
		# then calc the projection
		curr_vect = self.diff2DPointsToTuple(start,curr_point)
		curr_proj = self.dot2DTuple(curr_vect,goal_vect)
		# and angle deviation
		curr_angle = self.angleFrom2DTuple(curr_vect,goal_vect)
		# create score of projection times inverse angle deviation
		# to reward same angle and same distance
		curr_score = curr_proj * (2*math.pi - curr_angle)
		# if the best one yet update else continue
		if curr_score > best_score:
		    best_score = curr_score
		    best_goal = curr_point
	return best_goal

if __name__ == '__main__':
    rospy.init_node("ControllerNode")
    node = ControllerNode()
    rospy.spin()
