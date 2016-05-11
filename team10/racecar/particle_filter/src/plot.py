#!/usr/bin/env python
import rospy
import message_filters
import threading
import math
import random
import time
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
from time import sleep
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray

class Plot:
    def __init__(self):
	self.count = 0
        rospy.Subscriber("/racecar/mcl/motion_update", PoseArray, self.plot) # from sensorupdate
    
    def plot(self,data):
        self.count+=1
        if self.count>1000:
          self.count=0
          ylist=[]
          xlist=[]
          for i in data.poses:
               xlist.append(i.orientation.x)
               ylist.append(i.orientation.y)
	  print xlist
          plt.plot(xlist, ylist, 'ro')
          plt.show()
          time.sleep(2)
          plt.close('all')

if __name__ == "__main__":
    rospy.init_node('Plot', anonymous=True)
    Plot()
    # enter the ROS main loop
    rospy.spin()




