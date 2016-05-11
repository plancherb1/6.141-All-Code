#!/usr/bin/env python
import rospy
import itertools
import threading
import time
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

class LeftRightController:

	def __init__(self):
		self.curr_angle = None
		self.curr_dist = None
		self.curr_cone_bool = None
		self.past_angle = None
		self.past_dist = None
		self.past_cone_bool = None
		rospy.Subscriber('/racecar/object/position', Float64MultiArray, callback_pose)
		rospy.Subscriber('/racecar/image/internal_left_right_list', Float64MultiArray, callback_past_data)
		self.pub_list = rospy.Publisher('/racecar/image/internal_left_right_list', Float64MultiArray, queue_size=1)
		self.pub_bool = rospy.Publisher('/racecar/image/left_right_bool', Bool, queue_size=1)

		self.lock_pose = threading.Lock()
		self.lock_last_iteration = threading.Lock()
		self.lock_cone_side = threading.Lock()

	def callback_pose(self, data):
		with self.lock_pose:
			self.angle = data.data[0]
			self.dist = data.data[1]

	#Data.data defined as [prev_cone_bool, prev_angle, prev_distance,]
	def callback_past_data(self, data):
		with self.lock_last_iteration:
			self.past_cone_bool = data.data[0]
			self.past_angle = data.data[1]
			self.past_dist = data.data[2]
		self.cone_side()

	def cone_side(self):
		#initiate Float64MultiArray object for publishing
		newData = Float64MultiArray()
		#initiate Bool object for publishing
		newBool = Bool()

		#check if new cone has been detected:
		#First check if there is a sudden change in cone angle
		if self.curr_angle > self.past_angle+15 or self.curr_angle < self.past_angle-15:
			change_angle = True 
		else:
			change_angle = False

		#Next check if there is a sudden change in cone distance
		if self.curr_dist > self.past_dist+.2 or self.curr_angle < self.past_angle-.2:
			change_dist = True
		else:
			change_dist = False

		change_cone = change_angle or change_dist

		#if angle and distane have changed dramatically, flip cone_bool
		if change_cone:
			self.curr_cone_bool = not self.past_cone_bool
		else:
			self.curr_cone_bool = self.past_cone_bool

		#set new data equal to Float64MultiArray object.data
		newData.data = [self.curr_cone_bool, self.curr_angle, self.curr_dist]
		#set new data equal to Bool object
		newBool = self.curr_cone_bool

		#publish Floar64MultiArray object
		self.pub_list.publish(newData)
		self.pub_bool.publish(newBool)


	if __name__ == '__main__':
		rospy.init_node('LeftRightController', anonymous=True)
		rospy.spin()
