#!/usr/bin/env python
import rospy
import matplotlib.pyplot as pyplot
import mapping.read_pgm as mapper
import sensor_update
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point # float 64 x,y,z
from main_controller.msg import OccupancyGrid2
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class Grid:
    def __init__(self, resolution):
        rospy.init_node('occupancy_grid', anonymous=True)
        self.grid = rospy.Publisher('racecar/occupancy_grid',OccupancyGrid2, queue_size = 1 )
        rospy.Subscriber("racecar/laser/objdetect", Float64MultiArray, self.find_object)    
        self.resolution=resolution
        self.grid_side_length=10.0/resolution
        self.actual_grid_side_length=10.0
        self.current_grid=np.array([[0 for i in range(resolution)] for i in range(resolution)])
    
    def find_object(self, data):
        #print "recieved data"
        numItems = len(data.data)/3
        start = data.data[0:numItems]
        end = data.data[numItems:2 * numItems]
        lidar_distances = data.data[2 * numItems: 3 * numItems]
        for i in range(numItems):
            #print int(start[i]),int(end[i]), " angles"
            for angle in range(int(start[i]),int(end[i])+1):
                x=lidar_distances[i]*math.sin(angle*math.pi/180)
                y=lidar_distances[i]*math.cos(angle*math.pi/180)
                print x ,"  ",y
                x_grid=int(round(min(x/self.actual_grid_side_length*self.resolution+self.resolution/2.0,self.resolution-1)))
                y_grid=int(round(min(y/self.actual_grid_side_length*self.resolution+self.resolution/2.0,self.resolution-1)))
                self.update_grid(x_grid,y_grid)
        #np.savetxt("output.txt", self.current_grid)
        #UNCOMMENT 2 LINES BELOW FOR PYPLOT
        # pyplot.imshow(self.current_grid, pyplot.cm.gray)
        # pyplot.show()
        occupancy_grid = OccupancyGrid2()
        occupancy_grid.grid.data = self.current_grid.flatten().tolist()
        occupancy_grid.length = self.resolution
        occupancy_grid.width = self.resolution
        occupancy_grid.meters = self.actual_grid_side_length
        occupancy_grid.start = Point(float(self.resolution)/self.actual_grid_side_length/2,0,math.pi/2)
        occupancy_grid.goal = Point(0,0,0)

        self.grid.publish(occupancy_grid)
        #pyplot.imshow(self.current_grid, pyplot.cm.gray)
        #pyplot.show()

    def update_grid(self, x, y):
              for i in range(x-1,x+1):
                  for j in range(y-1,y+1):
                      self.current_grid[i][j] = 1


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    #rospy.init_node('Grid', anonymous=True)

    try:
        resolution=100
        Grid(resolution)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
