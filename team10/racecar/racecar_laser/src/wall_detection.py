#!/usr/bin/env python
import rospy
import message_filters
import threading
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray

class WallFinder:
    def __init__(self):
        self.left = None
        self.right = None
        rospy.Subscriber("racecar/laser/scan/left", Float64MultiArray, self.find_left_wall_callback)
        rospy.Subscriber("racecar/laser/scan/right", Float64MultiArray, self.find_right_wall_callback)
        
        self.lock_left = threading.Lock()
        self.lock_right = threading.Lock()
        
        
    def find_left_wall_callback(self, data):
        with self.lock_left:
            self.left = data.data
            
    def find_right_wall_callback(self, data):
        with self.lock_right:
            self.right = data.data
        self.find_wall()
        
        
    def find_wall(self):
        with self.lock_left:
            data_left = self.left
            data_right = self.right
        print "beginning"
        if data_left == None or data_right == None:
            print "either left or right is none"
            return
            
            
        #list of length 420 starting at 30 deg (1st element in array) and going to 135 deg (going counterclockwise)
        wall_detector_left = False
        min_distance_left = 10.0
        min_index_left = None
        scan_data_left = data_left
        for i in xrange(len(scan_data_left)-10):
            if sum(scan_data_left[i:i+10])/10.0 < min_distance_left:
                min_distance_left = min(sum(scan_data_left[i:i+10])/10.0, min_distance_left)
                min_index_left = i+5
        #min_angle is absolute angle - meaning that conforms to the -135 to 135 conventional layout of the rest of the code    
        #this is the angle from the forward pointing laser to the shortest distance to the wall     
        min_angle_left = 135-(len(scan_data_left)-1-min_index_left)*.25
        
        if min_distance_left != 10.0:
            wall_detector_left = True

        output_array_left = [wall_detector_left,min_distance_left,min_angle_left]
        
        
        #list of length 420 starting at -135 deg (1st element in array) and going to -30 deg (going counterclockwise)
        wall_detector_right = False
        min_distance_right = 10.0
        min_index_right = None
        scan_data_right = data_right
        for i in xrange(len(scan_data_right)-10):
            if sum(scan_data_right[i:i+10])/10.0 < min_distance_right:
                min_distance_right = min(sum(scan_data_left[i:i+10])/10.0, min_distance_right)
                min_index_right = i+5
        #min_angle is absolute angle - meaning that conforms to the -135 to 135 conventional layout of the rest of the code    
        #this is the angle from the forward pointing laser to the shortest distance to the wall     
        min_angle_right = -135+(len(scan_data_right)-1-min_index_right)*.25
        
        if min_distance_right != 10.0:
            wall_detector_right = True

        output_array_right = [wall_detector_right,min_distance_right,min_angle_right]
        output_array = output_array_left + output_array_right
        print output_array

       #print "out of loop!"
        pub_wall = rospy.Publisher('/racecar/laser/wall_detect', Float64MultiArray, queue_size=1)
        output = Float64MultiArray()
        output.data = output_array
        pub_wall.publish(output)


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node('wall_detection', anonymous=True)

    WallFinder();

    # enter the ROS main loop
    rospy.spin()




