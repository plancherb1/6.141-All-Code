#!/usr/bin/env python
import rospy
import message_filters
import threading
import math
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np



def meters_to_pixel(mapArray, resolution, x, y):
    new_x = max(min(round(x/resolution), len(mapArray[0][0]) - 1), 0)
    new_y = max(min(round(y/resolution), len(mapArray[0]) - 1), 0)
    return int(new_x), int(new_y)


class SensorUpdate:
    def __init__(self):
        self.laser_points = None
        self.particles = None
        self.laser_lock = threading.Lock()
        self.particle_lock = threading.Lock()
        self.map = self.getMap()
        self.angles = [0, 30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 240, 270, 300, 315, 330]
        self.num_angles = float(len(self.angles))
        self.resolution = 0.05
        self.update_weights = rospy.Publisher("racecar/mcl/update_weight", PoseArray)
        rospy.Subscriber("/scan", LaserScan, self.laser_update)
        rospy.Subscriber("racecar/laser/scan", LaserScan, self.laser_update)
        rospy.Subscriber("racecar/mcl/motion_update_particles", PoseArray, self.particles_update)
        rospy.Subscriber("racecar/mcl/init_particles", PoseArray, self.particles_init)

    def getMap(self):
        rawmap = np.loadtxt('mapping/map.txt')
        return rawmap.reshape((16, 312, 221))
       # print "OMG GOT MAP"
        #print len(self.map[0])
        #print len(self.map[0][0])

    def laser_update(self, data):
        #print 'receiving laser'
        with self.laser_lock:  
            self.laser_points = data.ranges
        

    def particles_update(self, data):
        print 'receiving other particles'
        with self.particle_lock:
            self.particles = data.poses
        self.sensor_update()

    def particles_init(self, data):
        print 'receiving init particles'
        update = False
        if self.particles == None:
            print 'initialize'
            with self.particle_lock:
                #print 'blah'
                self.particles = data.poses
                update = True
        if update:
            self.sensor_update()

    # *Borrowed from below, line 16
    # https://bitbucket.org/alexbuyval/ardroneum/src/fa5fac6070a2141e2a06f4f434acd942e3378f43/particles_filter/scripts/particlesfilter.py?at=master&fileviewer=file-view-default
    # This is just a gaussian kernel I pulled out of my hat, to transform
    # values near to robbie's measurement => 1, further away => 0
    def gauss(self, a, b):
        sigma2 = 0.1 ** 2
        error = a - b
        g = math.e ** -(error ** 2/ (2 * sigma2))
        return g


    def sensor_update(self):
        #print "updating"
        with self.laser_lock:
            #print "first lock"
            with self.particle_lock:
                #print "second lock"
                particles = self.particles
                new_laser_points = self.laser_points

        if new_laser_points == None or particles == None:
            print "either laser or particle data not received"
            return
        
        num_points = len(new_laser_points)
        print 'updating sensor ', len(particles)
        for particle in particles:
            x_meters = particle.orientation.x 
            y_meters = particle.orientation.y
            if (x_meters < 0 or y_meters < 0 or x_meters > 15.6 or y_meters > 11.05):
                particle.orientation.w = 0
            else:
                x, y = meters_to_pixel(self.map, self.resolution, x_meters, y_meters)
                theta = particle.orientation.z
                new_weight = 0
                minAngle = (theta - 270 / 2) % 360
                maxAngle = (theta + 270 / 2) % 360
                #iterate through the angles
                for z, angle in enumerate(self.angles):
                    if theta >= 135 and theta < 225:
                        acceptAngle = (angle > minAngle and angle < maxAngle)
                    else:
                        acceptAngle = angle > minAngle or angle < maxAngle
                    #find the angles that would be within the lidar range (-135 to 135)
                    if acceptAngle:
                        #convert from angle and theta to lidar index
                        index = round(num_points * ((angle - theta + 270 / 2) % 360) / 270.)
                        #check to make sure index is within range
                        index = int(max(min(index, num_points - 1), 0))
                        #(expected distance)
                        p_d = self.map[z][y][x]
                        #get the distance reported by the lidar
                        #(observed distance)
                        dist = new_laser_points[index]
                        #calculate weight based on difference of these, normalizing by number of angles
                        new_weight += self.gauss(p_d, dist)
                particle.orientation.w = new_weight
            
        #print 'gonna publish'
        #avgWeight = sum(particle.orientation.w for particle in particles)/float(len(particles))
        #particles = [particle for particle in particles if particle.orientation.w <  avgWeight/4.0]
        pointsToPublish = PoseArray()
        pointsToPublish.poses = particles
        self.update_weights.publish(pointsToPublish)

        #self.update_weights.publish(pointsToPublish)



if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    rospy.init_node('sensor_update', anonymous=True)

    SensorUpdate()

    # enter the ROS main loop
    rospy.spin()





