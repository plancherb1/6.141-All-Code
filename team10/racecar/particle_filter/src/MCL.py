#!/usr/bin/env python
import rospy
import message_filters
import threading
import math
import random
import time
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw
from time import sleep
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
import os
from multiprocessing import Process, Lock


def save_path(lock, path):
	#lock.aquire()
	np.save("path", path)
	#lock.release()

class MCL:
    def __init__(self,num_particles,width, height):
        plt.ion()
        self.points=None
        self.count=0
        self.mapWidth=width
        self.mapHeight=height
        self.num_particles=num_particles
        self.initialized = False
        self.map = self.load_map()
        self.angles = [0, 30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 240, 270, 300, 315, 330]
        self.resolution = 0.05
        self.makePoints()
        self.current_points = rospy.Publisher("/racecar/mcl/current_particles",PoseArray, queue_size = 10)
        self.motion_update_points = rospy.Publisher("/racecar/mcl/motion_update_particles", PoseArray,queue_size = 10)
        self.init_points = rospy.Publisher("/racecar/mcl/init_particles",PoseArray, queue_size = 10)
        rospy.Subscriber("/racecar/mcl/motion_update", PoseArray, self.motion_update)
        rospy.Subscriber("racecar/mcl/update_weight", PoseArray, self.sensor_update) # from sensorupdate
        rospy.Subscriber("/scan", LaserScan, self.start)
        rospy.Subscriber("racecar/laser/scan", LaserScan, self.start)
        #rospy.Subscriber("",,)
        self.path = np.array([])
        #this may be the error that you actually need to pass in the original Lock object 
        self.lock = Lock()

    def load_map(self):
    	rawmap = np.loadtxt('mapping/cropped_map.txt')
    	return rawmap.reshape((312, 221))

    def pixels_to_meter(self, x, y):
	    new_x = max(min(round(x * self.resolution), 15.6), 0)
	    new_y = max(min(round(y * self.resolution), 11.05), 0)
	    return new_x, new_y


    def sensor_update(self,data):
        if not self.count:
            print 'done initializing'
        self.count += 1
        particles = data.poses
        total_weight = sum(i.orientation.w for i in particles)
        for i in particles:
            i.orientation.w = i.orientation.w/float(total_weight)
        weights = np.array([i.orientation.w for i in particles])
        particles = [particle for particle in particles if particle.orientation.w > max(np.mean(weights) -  np.std(weights) - 0.1, 0.5/len(particles))]
        self.points.poses = particles
        print "max weight: ", max(i.orientation.w for i in self.points.poses), 'num weights: ', len(self.points.poses), 'avg weight: ', sum(i.orientation.w for i in self.points.poses)/float(len(self.points.poses)), 'min weight: ', min(i.orientation.w for i in self.points.poses)
        print "min x: ", min(i.orientation.x for i in self.points.poses), "max x: ", max(i.orientation.x for i in self.points.poses), "min y: ", min(i.orientation.y for i in self.points.poses), "max y: ", max(i.orientation.y for i in self.points.poses)
        if len(particles) <= 5 and len(particles) > 1:
            print "x: ", [i.orientation.x for i in self.points.poses]
            print "y: ", [i.orientation.y for i in self.points.poses]
            print "z: ", [i.orientation.z for i in self.points.poses]
            print "w: ", [i.orientation.w for i in self.points.poses]            
        #particle = max(particles, key = lambda p: p.orientation.w)
        point = [(particle.orientation.x, particle.orientation.y, particle.orientation.w) for particle in particles];
        if not len(self.path):
            self.path = np.array(point)
        else:
            self.path = np.vstack((self.path, point))
        #print self.path

        Process(target=save_path, args=(self.lock, self.path)).start()
        self.current_points.publish(self.points)


    def motion_update(self,data):
        self.initialized = True
        #print "recieved motion update data"
        self.points=data
        self.motion_update_points.publish(self.points)


    def start(self, data):
    	#print 'received laser data'
        if not self.initialized:
          print "starting the particle filter"
          self.current_points.publish(self.points) #initial publish to motion update
          self.init_points.publish(self.points)

    def makePoints(self):
    	print 'starting making points'
    	white_spots = np.transpose((self.map == 254).nonzero())
    	random_indices = np.random.randint(white_spots.shape[0],size = int(len(white_spots)*.1))
    	white_spots = white_spots[random_indices,:]
    	print len(white_spots)
    	#print white_spots
        self.points=PoseArray()
        arrayOfPoses=[]
        probabillity=1.0/self.num_particles
        blocks=int(math.sqrt(self.num_particles))
        xOffset=self.mapWidth/blocks
        yOffset=self.mapHeight/blocks
        for [i,j] in white_spots:
        	new_i, new_j = self.pixels_to_meter(i, j)
        	for k in self.angles:
        		pose=Pose(Point(0,0,0),Quaternion(new_i,new_j,k,probabillity))
        		arrayOfPoses.append(pose)
        self.points.poses=arrayOfPoses
        print 'done making points'
        #print self.points
        



            


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node('MCL', anonymous=True)

    MCL(6000,15.6,11.05)

    # enter the ROS main loop
    rospy.spin()




