#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
    
def parse_laser_data(data):
#	print "pre happy"
	#parsedData = LaserScan(data)
	angle_min = data.angle_min
	angle_max = data.angle_max
	num_readings = len(data.ranges)

#	print angle_min, angle_max, num_readings

	#note -- first entry is the right most, goes counter clockwise, middle entry is in front, last entry is furthest left
	#0-419 is right wall detection
	#420 - 659 is center range for detecting objects
	#660-1079 is for detecting left wall
	
	rightScan = data.ranges[0:180]
	right=Float64MultiArray()
	right.data=rightScan
   	centerScan = data.ranges[180:900] #[-55,55]
	center=Float64MultiArray()
	center.data=centerScan
	leftScan = data.ranges[900:1080]
	left=Float64MultiArray()
	left.data=leftScan
	
	pub_right_scan_data = rospy.Publisher('/racecar/laser/scan/right', Float64MultiArray, queue_size=1)
	pub_center_scan_data = rospy.Publisher('/racecar/laser/scan/center', Float64MultiArray, queue_size=1)
	pub_left_scan_data = rospy.Publisher('/racecar/laser/scan/left', Float64MultiArray, queue_size=1)

	pub_right_scan_data.publish(right)
	pub_center_scan_data.publish(center)
	pub_left_scan_data.publish(left)
	
	#print len(data.ranges)

def laser_parser():

    rospy.init_node('laser_parser', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, parse_laser_data)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_parser()
    except rospy.ROSInterruptException:
        pass

