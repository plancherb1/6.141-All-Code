#!/usr/bin/env python
import rospy
import itertools
import threading
import time
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped



class IBSController:

    def __init__(self):
        rospy.Subscriber('/racecar/image/data', Float64MultiArray, self.control)
        self.control_pub = rospy.Publisher("/vesc/racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)


   
    def control(self,data):
        car_ctl = AckermannDrive()
        car_msg = AckermannDriveStamped()
        [centerX,centerY,width,height,actual_angle]=data.data
        if actual_angle != -1 and width < 180 and height < 250:
            desired_angle = 0
            steering_gain=.4
            steering_range = .34
            error = desired_angle - actual_angle
            steering_angle = steering_gain * error
            car_ctl.speed = 0.3
            car_ctl.steering_angle = steering_angle
            car_msg.header.stamp = rospy.Time.now()
            car_msg.header.frame_id = "IBSController"
            car_msg.drive = car_ctl
            self.control_pub.publish(car_msg)
        else:
            car_ctl.speed = 0.0
            car_msg.header.frame_id = "IBSController"
            car_msg.drive = car_ctl
            self.control_pub.publish(car_msg)

        




if __name__ == '__main__':
    rospy.init_node('IBSController', anonymous=True)
    IBSController()
    rospy.spin()
