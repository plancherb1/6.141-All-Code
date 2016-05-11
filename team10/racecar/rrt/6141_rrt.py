#!/usr/bin/env python

#6.141 RRT

#import pygame
import rospy
import math
import random
import numpy as np
import scipy
import matplotlib.pyplot as plt 
#from scipy.integrate import odeint
from math import pi
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point # float 64 x,y,z
from main_controller.msg import OccupancyGrid2
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
import time

class RRT():
	def __init__(self):
		print "init!"
		#ROS intializations
		rospy.init_node('rrt_path', anonymous=True)
		print "init node"
		#self.grid = rospy.Publisher('racecar/occupancy_grid',OccupancyGrid, queue_size = 1 )
		#rospy.Subscriber("racecar/occupancy_grid", OccupancyGrid, self.main)   
		rospy.Subscriber("racecar/occupancy_grid_final", OccupancyGrid2, self.main) 
		self.controls = []
		self.path = None
		self.control_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
		print "init subscribed" 
		#desired graph cell size = 0.1m
		#desired grid size = 

		#CONTROLS:
		self.CONTROL_SEQUENCE = []

		#MAP SETUP
		#for pygame build use these parameters:
		self.GRID_LENGTH = 10.0
		self.GRID_SIZE = 100 #number of grid cells in x and y
		self.GRID_STEP =  self.GRID_LENGTH/self.GRID_SIZE #size of each grid square in m

		#for plotlib use these parameters:
		#self.GRID_STEP = .1 #size of each grid square in m
		#self.GRID_SIZE = 100 #number of grid cells in x and y

		self.BIAS_FACTOR = 0.25 #how often to sample goal state as random pt
		self.MAP_SIZE = math.floor(self.GRID_STEP*self.GRID_SIZE) #size of grid in m
		self.margin = 1 #self.margin for grid matrix (self.BLACK separating lines)
		self.SPARSITY = .97 #% of graph unoccupied

		#robot features
		self.ROBOT_LENGTH = .3 #the length of the robot is about 18 inches, or about .5 meters we think
		self.MAX_STEER = 0.34 #maximum steering angle of robot in rad
		self.MIN_STEER = -0.34 #minimum steering angle of robot in rad
		self.EXT_TIME= 4 #extension time for extending arc with dynamics in sec
		self.EXT_VEL = 0.25 #extension velocity for extending arc with dynamics in m/s
		self.SMALL_STEP_SIZE = 0.1 #size of each step within extension to check for collisions

		#colors for visualization
		self.BLACK = (0, 0, 0)
		self.WHITE = (255, 255, 255)
		self.GREEN = (0, 255, 0)   
		self.RED = (255, 0, 0) 
		self.YELLOW = (255,255,0)
		self.BLUE = (0,191,255)
		self.PINK = (255,192,203)
		self.PURPLE =  (148,0,211)

		#% of graph unoccupied
		self.SPARSITY = .97

	# def def1(self, ocpgrid):
	# 	print ocpgrid.data

	#converts coordinates from [m,m] to index on grid #49,2 -->
	def coord_to_index(self,pt): 
		x = (pt[0]/self.GRID_STEP)
		y = (pt[1]/self.GRID_STEP) 
		x_floor = math.floor(x)-1
		y_floor = math.floor(y)-1
		# if x_floor == self.GRID_SIZE:
		# 	x_floor = self.GRID_SIZE-1
		# if y_floor == self.GRID_SIZE:
		# 	y_floor = self.GRID_SIZE -1
		# if x_floor >= self.GRID_SIZE or y_floor >= self.GRID_SIZE:
		# 	#print x_floor, y_floor
		# 	return "coordinate out of range!"
		# else:
		index = int(y_floor*self.GRID_SIZE+x_floor)
		return index

	#returns euclidean self.distance between index1 and index2 start_pt and end_pt
	def distance(self,pt1, pt2):
		return math.sqrt(((pt2[0]-pt1[0])*(pt2[0]-pt1[0])+(pt2[1]-pt1[1])*(pt2[1]-pt1[1])))

	#returns 3D euclidean distance between 2 points
	def distance_3D(self,pt1, pt2):
		return math.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2+(pt2[2]-pt1[2])**2)

	def basic_grid_generator2(self,GRID_SIZE_x, GRID_SIZE_y):
		grid = []
		#assign occupancy values to grid
		occupied_coords = []
		i = 10
		j = 10
		while(i <= 15):
			while (j <=30):
				occupied_coords.append([i,j])
				j = j+.5
			i = i+0.5
			j = 10

		#add another obstacle
		a = 25
		b = 25
		while(a <= 30):
			while (b < 40):
				occupied_coords.append([a,b])
				b = b+.5
			a = a+0.5
			b = 25

		c = 35
		d = 5
		while(c <= 40):
			while (d <=15):
				occupied_coords.append([c,d])
				d = d+.5
			c = c+0.5
			d = 5
		#convert occupied_coords to occupied_indices
		occupied_indices = []
		for i in occupied_coords:
			occupied_indices.append(self.coord_to_index(i))
		#generate grid
		for i in range(0,GRID_SIZE_x*GRID_SIZE_y):
			if i in occupied_indices:
				grid.append(1)
			else:
				grid.append(0)
		return grid

	#random point generator - generate random points in continuous space within grid size ... only generates random x,y coordinates
	def rand_pt(self,end_state, grid):
	    perct = np.random.random()
	    #bias point to goal region self.BIAS_FACTOR% of time
	    if perct <=self.BIAS_FACTOR:
	        [x_rand, y_rand] = end_state
	    else:
	        x_rand = np.random.random()*(self.GRID_SIZE-1)*self.GRID_STEP
	        y_rand = np.random.random()*(self.GRID_SIZE-1)*self.GRID_STEP
	        while (grid[self.coord_to_index([x_rand, y_rand])] == 1):
	       		x_rand = np.random.random()*(self.GRID_SIZE-1)*self.GRID_STEP
	        	y_rand = np.random.random()*(self.GRID_SIZE-1)*self.GRID_STEP
	    return [x_rand, y_rand]

	 #determines nearest point in tree to pt based on 3D euclidean distance and generated theta
	def near_pt_3D(self, pt, tree):
		x_1,y_1 = pt #randonly selected point
		point = tree[0]
		x_0 = tree[0][0]
		y_0 = tree[0][1]
		theta_0 = tree[0][2]
		#calculate angle of line between current pt and random pt
		delta_x = x_1-x_0
		delta_y = y_1-y_0
		theta = math.atan2(delta_y,delta_x)
		#print theta
		#initialize minimum self.distance
		min_dist = self.distance_3D([x_1,y_1,theta], [x_0,y_0,theta_0]) 

		for i in tree[1:len(tree)]:
			x_0 = i[0]
			y_0 = i[1]
			theta_0 = i[2]
			delta_x = x_1-x_0
			delta_y = y_1-y_0
			#print delta_x,delta_y
			theta = math.atan2(delta_y,delta_x)
			#print theta
			new_dist = self.distance_3D([x_1,y_1,theta], [x_0,y_0,theta_0])
			if (new_dist) < min_dist:
				min_dist = new_dist
				point = i
		return point


	def determine_steering(self, start_state, end_state, gain=1):
		#start_state has theta since it is from the tree, end_state does not, since it's a potential goal state
		#these thetas are oriented such that 0 is along the x-axis. They will have to be changed when coming into the class & when leaving
		x_int = start_state[0]
		y_int = start_state[1]
		theta = start_state[2]
		[x_final, y_final] = end_state
		r_eucl = self.distance([x_int, y_int],[x_final, y_final])
		delta_x = x_final - x_int
		#print "delta_x", delta_x
		delta_y = y_final - y_int
		#print "delta_y", delta_y
		phi_diff = math.atan2(delta_y,delta_x)

		phi_diff = (phi_diff - theta)%(2*pi)
		if phi_diff > pi:
			phi_diff = phi_diff-2*pi
		if phi_diff < -pi:
			phi_diff = phi_diff +2*pi
		#print "phi_diff", phi_diff
		if phi_diff > pi/2.0:
			return self.MAX_STEER
		elif phi_diff < -pi/2.0:
			return  self.MIN_STEER 
		elif math.sin(phi_diff) == 0:
			return 0
		else:
			radius = r_eucl/(2*math.sin(phi_diff))
			angle = gain*self.ROBOT_LENGTH*1.0/radius # in radians
			angle_final = min(self.MAX_STEER, max(self.MIN_STEER, angle))
			return angle_final


	def racecar_model(self, start_state,steering_angle, v, small_delta_t): #v=lin_vel, rhodot=steering velocity, L is span of car
		#set constants

		rho = steering_angle
		L = 1
		#set y equal to controls
		#print states
		x = start_state[0]
		y = start_state[1]
		theta = start_state[2]
		#check theta bounds
		if theta > np.pi:
			theta = theta - 2*np.pi
		if theta < -np.pi:
			theta = theta + 2*np.pi

		xdot = np.cos(theta)*v*np.cos(rho) 
		ydot = np.sin(theta)*v*np.cos(rho) 
		thetadot = (np.tan(rho)/L)*v*np.cos(rho)

		x_new = x + small_delta_t*xdot
		y_new = y + small_delta_t*ydot
		theta_new = theta+ small_delta_t*thetadot
		if theta_new > np.pi:
			theta_new = theta_new - 2*np.pi
		if theta_new < -np.pi:
			theta_new = theta_new + 2*np.pi
		return [x_new, y_new, theta_new]
 

	def extend_pt_arc(self,start_state, delta_t, steering_angle, vel):
		#use the steering angle to reach end_state and follow arc for delta_t
		#let's determine the equation of the circle from the start_state, radius, and note that the
		#let's say we wannt go 10 cm <333333 per little baby tiny step
		#so let's determine the distance = vt
		distance = math.ceil(delta_t*vel*10)/10.0
		#how many times to split apart?
		#num_iterations = int(distance/.1)
		num_iterations = int(distance/self.SMALL_STEP_SIZE)
		state_list = [start_state]
		next_state = start_state
		small_delta_t = 1.0*delta_t/num_iterations

		for i in range(num_iterations):
			#go through getting all the states
			next_state = self.racecar_model(next_state,steering_angle, vel, small_delta_t)
			#cap next state to inside of map
			# next_state[0] = max(min(next_state[0],self.MAP_SIZE),0)
			# next_state[1] = max(min(next_state[1],self.MAP_SIZE),0)
			#return "out of range" if next_state is off the grid
			if next_state[0] >= self.MAP_SIZE or next_state[0] <= 0 or next_state[1] >= self.MAP_SIZE or next_state[1] <= 0:
				return "out of range"
			else:
				state_list.append(next_state)
		#state list is a list of [x,y,theta] indices [m,m,rad] in global reference
		return state_list


	#returns nearest point in tree to pt all where pt in grid_coord
	def near_pt(self,pt, tree):
	    min_dist = self.distance(pt, tree[0]) 
	    point = tree[0]
	    for i in tree[1:len(tree)]:
	        new_dist = self.distance(pt, i)
	        if (new_dist) < min_dist:
	            min_dist = new_dist
	            point = i
	    return point


	#takes in list of states to check, converts them to indices in grid and checks if trajectory passes through occupied cells
	#returns True if collision free, False if not
	def collision_free(self,states, grid):
		#print states
		for i in states:
			x = i[0]

			y = i[1]
			#print x, y
			index = self.coord_to_index([x,y])
			if grid[index] == 1:
				return False
		return True

		
	#returns continuous path (array of pts) from first node in memory to final node in memory (memory stores parent node)
	def find_path(self,memory, tree):
	    rev_path = []
	    goal = tree[-1]
	    start = tree[0]
	    rev_path.append(goal)
	    i = 0;
	    while not(rev_path[-1] == start):
	        #find index of ith point of path in tree
	        index = tree.index(rev_path[i])
	        rev_path.append(memory[index-1])
	        i+=1
	    return rev_path[::-1]
		

	def RRT_visualize_build_racecar(self,start_state, end_state, end_region_radius, grid, X_DISP, Y_DISP):
		counter = 0
		XRESOL = X_DISP/self.GRID_SIZE
		YRESOL = Y_DISP/self.GRID_SIZE
		pygame.init()
		size = (X_DISP, Y_DISP)
		screen = pygame.display.set_mode(size)
		pygame.display.set_caption("GRID")
		done = False
		clock = pygame.time.Clock()
		tree = [start_state]
		end_coords = [end_state]
		for i in range(0,10):
			for j in range(0,10):
				#goal_coords.append([goal[0]-i*self.GRID_STEP, goal[1]-j*self.GRID_STEP])
				end_coords.append([end_state[0]-i*self.GRID_STEP, end_state[1]-j*self.GRID_STEP])
		memory = []
		while not done:
		    # --- Main event loop
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					done = True
			screen.fill(self.WHITE)
			#Draw start location
			pygame.draw.rect(screen, self.RED, [math.floor(start_state[0]/self.GRID_STEP)*XRESOL, math.floor(start_state[1]/self.GRID_STEP)*YRESOL, XRESOL, YRESOL], 0)
			#Draw end region
			for i in end_coords:
				pygame.draw.rect(screen, self.GREEN, [math.floor(i[0]/self.GRID_STEP)*XRESOL, math.floor(i[1]/self.GRID_STEP)*YRESOL, XRESOL, YRESOL], 0)
			#Draw grid lines  --> comment this out if square size ~ )1 pixel (or else entire screen will appear self.BLACK)
			for i in range(0,self.GRID_SIZE):
				for j in range(0,self.GRID_SIZE):
					pygame.draw.rect(screen, self.BLACK, [XRESOL*i,YRESOL*j,XRESOL,YRESOL], self.margin)
			#Draw obstacles
			for i in range(0,len(grid)):
				if grid[i] == 1:
					pygame.draw.rect(screen, self.BLACK, [(i%self.GRID_SIZE)*XRESOL,(i/self.GRID_SIZE)*YRESOL,XRESOL,YRESOL], 0)

			#RRT algorithm with dynamics
			while (self.distance([tree[-1][0], tree[-1][1]], end_state) > end_region_radius):
				##RRT calculations
				rand = self.rand_pt(end_state, grid)
				#check if rand already in tree ... if so recalculate rand
				for i in tree:
					while(i[0] == rand[0] and i[1] == rand[1]):
						rand = self.rand_pt(end_state, grid)
			
				#calculate nearest point in tree to randomly generate point (indlucing dynamics)
				near_pt = self.near_pt_3D(rand, tree)

				#near_index = coord_to_index(near_pt)
				pygame.draw.rect(screen, self.BLUE, [math.floor(near_pt[0]/self.GRID_STEP)*XRESOL, math.floor(near_pt[1]/self.GRID_STEP)*YRESOL, XRESOL, YRESOL], 0)

				#extend near towards rand using optimal control policy
				#set constants and extend arc using dynamics.
				steering_angle = self.determine_steering(near_pt, rand, gain=1)
				new_states = self.extend_pt_arc(near_pt, self.EXT_TIME, steering_angle, self.EXT_VEL)
				#draw new on grid
				new = new_states[-1]
				pygame.draw.rect(screen, self.PINK, [math.floor(new[0]/self.GRID_STEP)*XRESOL, math.floor(new[1]/self.GRID_STEP)*YRESOL, XRESOL, YRESOL], 0)

				#check if path from near to new collision free ... if occupied, continue.If free, add new to tree
				#check if trajectory to new is collision free
				if (self.collision_free(new_states, grid)):
					tree.append(new_states[-1])
					#append parent state to memory list
					memory.append(near_pt)
				# else:
				# 	#print 'collision!'
				counter = counter+1
				pygame.display.flip()
			path = self.find_path(memory, tree)
			for i in path:
				pygame.draw.rect(screen, self.BLUE, [math.floor(i[0]/self.GRID_STEP)*XRESOL, math.floor(i[1]/self.GRID_STEP)*YRESOL, XRESOL, YRESOL], 0)
			pygame.display.flip()

	#build RRT path and display path along with obstacles in plotlib
	def RRT_racecar(self,start_state, end_state, end_region_radius, grid):
		print 'RRT method'
		counter = 0
		done = False
		tree = [start_state]
		tree_with_steering = [start_state]
		prev_control = None
		control_tree = [prev_control]
		end_coords = [end_state]
		for i in range(0,10):
			for j in range(0,10):
				end_coords.append([end_state[0]-i*self.GRID_STEP, end_state[1]-j*self.GRID_STEP])
		path_memory = []
		control_memory = []

		#RRT algorithm with dynamics
		while (self.distance([tree[-1][0], tree[-1][1]], end_state) > end_region_radius):
			#print 'iteration'
			##RRT calculations
			rand = self.rand_pt(end_state, grid)
			#check if rand already in tree ... if so recalculate rand
			for i in tree:
				while(i[0] == rand[0] and i[1] == rand[1]):
					rand = self.rand_pt(end_state, grid)
		
			#calculate nearest point in tree to randomly generate point (indlucing dynamics)
			near_pt = self.near_pt_3D(rand, tree)
			#set constants and extend arc using dynamics.
			steering_angle = self.determine_steering(near_pt, rand, gain=1)
			new_states = self.extend_pt_arc(near_pt, self.EXT_TIME, steering_angle, self.EXT_VEL)
			#make sure point extension is not off the grid
			while new_states == "out of range":
				rand = self.rand_pt(end_state, grid)
				#check if rand already in tree ... if so recalculate rand
				for i in tree:
					while(i[0] == rand[0] and i[1] == rand[1]):
						rand = self.rand_pt(end_state, grid)
				near_pt = self.near_pt_3D(rand, tree)
				steering_angle = self.determine_steering(near_pt, rand, gain=1)
				new_states = self.extend_pt_arc(near_pt, self.EXT_TIME, steering_angle, self.EXT_VEL)
			
			#take just the last value from the extended states
			new = new_states[-1]
			#to visualize "arcs" store all of the extended states plus steering angle
			# for i in range(len(new_states)):
			# 	new_states[i].append(steering_angle)
			# plt.plot(new[0],new[1])
			# plt.xlabel('x')
			# plt.ylabel('y')
			# plt.show()
			#print steering_angle

			#check if path from near to new collision free ... if occupied, continue.If free, add new to tree
			#check if trajectory to new is collision free
			if (self.collision_free(new_states, grid)):
				tree.append(new_states[-1])
				tree_with_steering[-1].append(steering_angle)
				#append entire arcs
				tree_with_steering.append(new)

				#curr_control = [steering_angle, self.EXT_VEL, self.EXT_TIME]
				#control_tree.append(curr_control)
				#append parent state to memory list
				path_memory.append(near_pt)
				#append parent control to memory
				#control_memory.append(prev_control)
				#prev_control = curr_control

			counter = counter+1
		path = self.find_path(path_memory, tree_with_steering)
		self.path = path
		#control_sequence = self.find_path(control_memory, control_tree)
		#self.controls = control_sequence
		#print path
		print len(path)
		#draw path in plotlib
		x_list = []
		y_list = []
		for state in path:
			x_list.append(state[0])
			y_list.append(state[1])
	        
  		
  	
   		x_obst = []
   		y_obst =[]
   		for i in range(len(grid)):
   		  	if grid[i] == 1:
   		  		x_obst.append((i%self.GRID_SIZE)*self.GRID_STEP)
   		  		y_obst.append((math.floor(i/self.GRID_SIZE))*self.GRID_STEP)
   		plt.plot(x_obst, y_obst,'*')
   		plt.hold

		plt.plot(x_list, y_list)
		plt.axis([0, 10, 0, 10])
		plt.xlabel('x')
		plt.ylabel('y')
		plt.show()
		

	def publisher(self):
		print "called"
                self.state=0
                self.newControl=AckermannDriveStamped()
                s=rospy.Timer(rospy.Duration(self.EXT_TIME),self.timerCallBack)
		while(self.state<len(self.path)-1):
                     self.control_pub.publish(self.newControl)
                self.state=0
                s.shutdown()

			#print "for loop"
			#reversing direction of steering angle to be compatible with hardware
			#in code, positive steer is left but on hardware positive steer is right

			
                        

        def timerCallBack(self,event):
                    state=self.path[self.state]
                    steer_angle = -1.*state[3]
		    vel = self.EXT_VEL
		    newControl = AckermannDriveStamped()
		    newDrive = AckermannDrive()
		    newDrive.steering_angle = steer_angle
		    newDrive.speed = vel
		    self.newControl.drive = newDrive
                    self.state+=1
                   
			

	# def grid(self,data):
	# 	return data.data

	def main(self,data):
		#set start and goal locations in meters
		print 'started'
		start = data.start
		start = [start.x,start.y,start.z]
		goal = data.goal
		goal = [goal.x,goal.y]
		end_region_radius = 0.5 #m radius within end location
		self.GRID_LENGTH = data.meters
		self.GRID_SIZE = data.length #number of grid cells in x and y
		self.GRID_STEP =  self.GRID_LENGTH/self.GRID_SIZE #size of each grid square in m
		self.MAP_SIZE = math.floor(self.GRID_STEP*self.GRID_SIZE)
		#generate random grid
		#grid = self.grid_generator(self.GRID_SIZE, self.GRID_SIZE, start, goal, self.SPARSITY)
		
		#print "grid generated"
		#grid = self.basic_grid_generator2(self.GRID_SIZE, self.GRID_SIZE)	
		#create grid from inputted occupancy_array
		grid = data.grid.data
		#print "path generated"
		#visualize build in pygame
		#self.RRT_visualize_build_racecar(start, goal, end_region_radius, grid, 800,800)
		#visualize build in plotlib
		self.RRT_racecar(start, goal, end_region_radius, grid)
		#print "RRT done"
		self.publisher()
		#print 'DONE!'

if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    #rospy.init_node('Grid', anonymous=True)

    try:
    	print "main"
        rrt1 = RRT()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
