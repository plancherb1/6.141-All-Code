#!/usr/bin/python
#
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray # array if poses
from geometry_msgs.msg import Pose # a position (point) and an orientation (Quaternion)
from geometry_msgs.msg import Point # float 64 x,y,z
from geometry_msgs.msg import Quaternion # float64 x,y,z,w

class MotionUpdateNode:
    def __init__(self):
        # subscribe to the odometry, maybe also subscribe to IMU and integrate to get better motion model?
        rospy.Subscriber("/vesc/odom", Odometry, self.MotionCallback) # on car
        rospy.Subscriber("/racecar/vesc/odom", Odometry, self.MotionCallback) # on simulator

        # subscribe to the current particles
        rospy.Subscriber("/racecar/mcl/current_particles", PoseArray, self.ParticleCallback)
        
        # advertise that we'll publish on the /racecar/motion_update topic
        self.MotionPub = rospy.Publisher("/racecar/mcl/motion_update", PoseArray, queue_size = 1)

        # keep the current and previous position
        self.prev_pos = (0,0,0)
        self.curr_pos = (0,0,0)

    def MotionCallback(self,odom_message):
        # continouously update the current position
        position = odom_message.pose.pose.position
        self.curr_pos = (position.x,position.y,position.z)

    def ParticleCallback(self,particles):
        #print "particle recieved by motionUpdate"
        poses = particles.poses
        # figure out how much we have moved
        temp_curr_pos = self.curr_pos
        curr_change = [a_i - b_i for a_i, b_i in zip(temp_curr_pos, self.prev_pos)]
	#print curr_change

        # get some random noise
        num_particles = len(poses)
        mu, sigma = 1, 0.1
        noise = np.random.normal(mu, sigma, num_particles*3)

        # update the particles applying noise
        for i in range(num_particles):
            poses[i].orientation.x = poses[i].orientation.x + (curr_change[0] * noise[3*i])
            poses[i].orientation.y = poses[i].orientation.y + (curr_change[1] * noise[3*i+1])
            poses[i].orientation.z = poses[i].orientation.z + (curr_change[2] * noise[3*i+2])

        # then publish the updated particles
        self.MotionPub.publish(particles)
        #print 'motionUpdate is publishing particles'

        # then set the prev_pos to temp_curr_pos to get ready for next process
        self.prev_pos = temp_curr_pos

if __name__ == '__main__':
    rospy.init_node("MotionUpdateNode")
    node = MotionUpdateNode()
    rospy.spin()
