#!/usr/bin/env python
import rospy
import matplotlib.pyplot as pyplot
import mapping.read_pgm as mapper
import sensor_update
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class Grid:
    def __init__(self, resolution):
        rospy.init_node('occupancy_grid', anonymous=True)
        rospy.Subscriber("racecar/laser/objdetect", Float64MultiArray, self.find_object)    
        self.resolution=resolution
        self.grid_side_length=10.0/resolution
        self.actual_grid_side_length=10.0
        self.current_grid=[[0 for i in range(resolution)] for i in range(resolution)]
    
    def find_object(self, data):
        #print "recieved data"
        numItems = len(data.data)/3
        start = data.data[0:numItems]
        end = data.data[numItems:2 * numItems]
        lidar_distances = data.data[2 * numItems: 3 * numItems]
        for i in range(numItems):
            #print int(start[i]),int(end[i]), " angles"
            list_of_points=[]
            for angle in range(int(start[i]),int(end[i])+1):
                x=lidar_distances[i]*math.cos(angle*math.pi/180)
                y=lidar_distances[i]*math.sin(angle*math.pi/180)
                x_grid=int(round(min(x/self.actual_grid_side_length*resolution/2.0+self.resolution/2.0,self.resolution)))
                y_grid=int(round(min(y/self.actual_grid_side_length*resolution/2.0+self.resolution/2.0,self.resolution)))
                list_of_points.append((x_grid,y_grid))
            self.update_grid(x_grid,y_grid)
        #np.savetxt("output.txt", self.current_grid)
        pyplot.imshow(self.current_grid, pyplot.cm.gray)
        pyplot.show()

    def update_grid(self, list_of_points):
         for list_of_points


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    #rospy.init_node('Grid', anonymous=True)

    try:
        resolution=500
        Grid(resolution)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
