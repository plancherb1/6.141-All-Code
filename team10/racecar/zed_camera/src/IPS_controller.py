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



class IPSController:
    
    def __init__(self):
        self.right=False
        rospy.Subscriber('/racecar/right_left', Bool, self.avoid) # go left or right around obstacle, true==right
        rospy.Subscriber('/racecar/object/position', Float64MultiArray, self.control) # [angle, distance]  maybe rename
        self.control_pub = rospy.Publisher("/vesc/racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
	
    def avoid(self,data):
		self.right=data.data

    def control(self,data):
        car_ctl = AckermannDrive()
        car_msg = AckermannDriveStamped()
        object_angle=data.data[0]*math.pi/180
        object_distance=data.data[1]
        if data.data[0] != -1: 
            desired = -.3 #how wide to clear cone
            multiplier=1
            if(not self.right):
				desired = .3
            desired_distance=.1
            distance_y = object_distance * math.cos(object_angle)
            if distance_y < .4:
		  desired *= 2./3
	    steering_gain=.1
            steering_max = .34
            actual=multiplier * math.sin(object_angle)*object_distance
	    #if actual < desired and object_distance > desired * 1.2:
	    #	actual = object_distance
            error=desired-actual
            print actual, " actual"
            print object_distance, " object_distance" 
            print steering_gain * error, " steering"
            steering_angle = multiplier*steering_gain * error

            car_max_speed=1.0
            #desired_distance=.1
	    #distance_y = object_distance * math.cos(object_angle)
            distance_error=distance_y-desired_distance
	    print 'distance y ', distance_y
            speed_gain=.4
            speed=distance_error*speed_gain
	    print 'speed ', speed
            #speed=.3
            car_ctl.speed = speed
            car_ctl.steering_angle = steering_angle
            car_msg.header.stamp = rospy.Time.now()
            car_msg.header.frame_id = "IPSController"
            car_msg.drive = car_ctl
            self.control_pub.publish(car_msg)
        else:
	    car_ctl.steering_angle = 0
            car_ctl.speed = 0 #change for, later crawl forward
            car_msg.header.frame_id = "IPSController"
            car_msg.drive = car_ctl
            self.control_pub.publish(car_msg)

        




if __name__ == '__main__':
    rospy.init_node('IPSController', anonymous=True)
    IPSController()
    rospy.spin()
