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
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

class IBSController:

    def __init__(self):
        self.last=0
        rospy.Subscriber('/racecar/controls/direction', Point, self.control)
        self.control_pub = rospy.Publisher("/racecar/move_commands", AckermannDriveStamped, queue_size=0)
        #self.sim_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=0)
        #self.steer= rospy.Subscriber("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)      
        self.max_steering_angle = 0.35
   
    def control(self,point):
        car_ctl = AckermannDrive()
        car_msg = AckermannDriveStamped()
        #[centerX,centerY,width,height,actual_angle]=data.data
        x, y, z = (point.x, point.y, point.z)
        if True:
            '''desired_angle = 0
            steering_gain=.4
            steering_range = .34
            error = desired_angle - actual_angle
            steering_angle = steering_gain * error'''
	    z = z/3
            steering_angle = self.last+(-z-self.last)/4.0
	    if abs(steering_angle) > self.max_steering_angle:
		if steering_angle < 0:
		    steering_angle = -1*self.max_steering_angle
		else:
		    steering_angle = self.max_steering_angle
            #steering_angle = -z;
            car_ctl.speed = 4.0
            car_ctl.steering_angle = steering_angle
            self.last=steering_angle
            car_msg.header.stamp = rospy.Time.now()
            car_msg.header.frame_id = "LaserController"
            car_msg.drive = car_ctl
        else:
            car_ctl.speed = 0.0
            car_msg.header.frame_id = "LaserController"
            car_msg.drive = car_ctl

        self.control_pub.publish(car_msg)
        #self.sim_pub.publish(car_msg)

        




if __name__ == '__main__':
    rospy.init_node('LaserController', anonymous=True)
    IBSController()
    rospy.spin()
