#!/usr/bin/env python
#class to expand obstacles:
import rospy
import math
import matplotlib.pyplot as plt 
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point # float 64 x,y,z
from main_controller.msg import OccupancyGrid2
import numpy as np


class ObstacleExpansion():
	def __init__(self):
		rospy.init_node('obstacle_expansion', anonymous=True)
		rospy.Subscriber("racecar/occupancy_grid", OccupancyGrid2, self.expand_obstacles) 
		self.expanded_grid = rospy.Publisher("racecar/occupancy_grid_expanded", OccupancyGrid2, queue_size = 1)
		
		self.grid = []
		self.GRID_SIZE = 100
		self.GRID_STEP = 0.5
		self.ROBOT_LEN = 1.0

	#method that expands obstacles in grid by length of robot (assume robot is a square)
	def expand_obstacles(self,data):
		print "expanding obstacles"
		#initialize copy of grid
		new_grid = []
		self.GRID_SIZE = data.length
		self.GRID_STEP = data.meters
		self.grid = data.grid.data
		for i in range(len(self.grid)):
			new_grid.append(self.grid[i])
		print type(new_grid)

		cells_to_expand = int(math.ceil(self.ROBOT_LEN/self.GRID_STEP))
		print cells_to_expand
		for i in range(len(self.grid)):
			if self.grid[i] == 1:
				for j in range(1,cells_to_expand):
					#set right and left indices as occupied
					right = min(max(i+j,0),self.GRID_SIZE*self.GRID_SIZE-1)
					left = min(max(i-j,0),self.GRID_SIZE*self.GRID_SIZE-1)

					new_grid[right] = 1
					new_grid[left] = 1

					down = min(max(i-self.GRID_SIZE*j,0),self.GRID_SIZE*self.GRID_SIZE-1)
					up = min(max(i+self.GRID_SIZE*j,0),self.GRID_SIZE*self.GRID_SIZE-1)
					new_grid[down] = 1
					new_grid[up] = 1


		# x_obst = []
  # 		y_obst =[]
  # 		for i in range(len(new_grid)):
  # 			if new_grid[i] == 1:
  # 				x_obst.append((i%self.GRID_SIZE)*self.GRID_STEP)
  # 				y_obst.append((math.floor(i/self.GRID_SIZE))*self.GRID_STEP)
  # 		plt.plot(x_obst, y_obst, '*')
  # 		plt.axis([0, 50, 0, 50])
  # 		plt.show()
  		#print new_grid

  		# add the new grid back to the original data and send
  		data.grid.data = new_grid
  		self.expanded_grid.publish(data)


# if __name__ == '__main__':
# 	rrt1 = RRT()
# 	grid1 = rrt1.basic_grid_generator2(100,100)

# 	new_grid = ObstacleExpansion(grid1)
# 	new_grid.expand_obstacles()
# 	print 'expaned'

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    #rospy.init_node('Grid', anonymous=True)

    try:
    	print "main"
        expand_grid = ObstacleExpansion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


