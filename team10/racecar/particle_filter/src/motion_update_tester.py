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
from geometry_msgs.msg import PoseWithCovariance # pose and covariance
from geometry_msgs.msg import Vector3 #x,y,z floats
from geometry_msgs.msg import Twist # linar and angular vector3s
from geometry_msgs.msg import TwistWithCovariance # twist and covariance

class MotionUpdateNodeTester:
    def __init__(self):
        # publish the odometry
        self.OdomPub = rospy.Publisher("/vesc/odom", Odometry, queue_size = 10) # on car

        # Publish the current particles
        self.ParticlePub = rospy.Publisher("/racecar/mcl/current_particles", PoseArray, queue_size = 10)
        
        # Subscribe to the answer
        rospy.Subscriber("/racecar/motion_update", PoseArray, self.AnswerCheck)

        # create our starting point for particles and publish
        self.initial_particles = PoseArray()
        a = Pose(Point(0,0,0),Quaternion(0,0,0,0))
        b = Pose(Point(0,0,0),Quaternion(1,1,1,0))
        c = Pose(Point(0,0,0),Quaternion(-1,-1,-1,0))
        self.initial_particles.poses = [a,b,c]

        # also do it for odometry
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

        # then start publishing odometry and update
        rate = rospy.Rate(1) # 1hz
        flag = True
        while not rospy.is_shutdown():
            a = Pose(Point(self.current_x,self.current_y,self.current_z),Quaternion(0,0,0,0))
            cov = [0]*36
            b = PoseWithCovariance(a,cov)
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose = b
            odom_msg.twist = TwistWithCovariance(Twist(Vector3(0,0,0),Vector3(0,0,0)),cov)
            self.OdomPub.publish(odom_msg)
            rate.sleep()
            # then increase our x and y and z by 1 and do it again
            self.current_x += 1
            self.current_y += 1
            self.current_z += 1
            if flag:
                print self.initial_particles.poses[0].orientation
		print
                self.ParticlePub.publish(self.initial_particles)
                flag = False

    def AnswerCheck(self,particles):
        print particles.poses[1].orientation
        # publish again
        self.ParticlePub.publish(particles)

if __name__ == '__main__':
    rospy.init_node("MotionUpdateNodeTester")
    node = MotionUpdateNodeTester()
    rospy.spin()
