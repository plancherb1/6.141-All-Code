#!/usr/bin/python
#
import rospy
import math
from racecar_nav.msg import IsObstacle
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 
from pid import PID

class SafetyNode:
    def __init__(self):
        self.safetydistance = 0.5
        self.linear_scale = 2.0
        self.angular_scale = 2.0
        # subscribe to the IsObstacle topic
        rospy.Subscriber("IsObstacle", IsObstacle, self.safety_callback)
        rospy.Subscriber("LeftSideAngle", Float32, self.left_wall_callback)
        rospy.Subscriber("RightSideAngle", Float32, self.right_wall_callback)
        # advertise that we'll publish on the cmd_vel topic for ackermann_mux_cmd
        self.cmd_vel_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)
        self.wall_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
        # """ Publish to Simulator """
        # advertise that we'll publish on the cmd_vel topic for ackermann_mux_cmd
        # self.cmd_vel_pub = rospy.Publisher("/racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=10) # TODO 
        # advertise that we'll publish on the ackermann angle topic for ackermann_mux_cmd
        # self.wall_pub = rospy.Publisher("/racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=10)
        
        #create pid controller
        self.p_gain = 1
        self.i_gain = 3.5
        self.d_gain = 0
        self.i_max = 0
        self.i_min = 0
        self.pid_controller = PID(self.p_gain,self.i_gain,self.d_gain,self.i_max,self.i_min)
        
        # send initial start velocity command
        #global frame_id
        #global wheelbase
        #msg = AckermannDriveStamped()
        ##msg.header.stamp = rospy.Time.now()
        #msg.header.frame_id = frame_id
        #msg.drive.steering_angle = 0.0
        #msg.drive.speed = 1.0
        #self.cmd_vel_pub.publish(msg)


    def safety_callback(self, IsObstacle_msg):
        global frame_id
        global wheelbase
        msg = AckermannDriveStamped()
        #msg.header.stamp = rospy.Time.now()
        #msg.header.frame_id = frame_id
        msg.drive.steering_angle = 0.0
        # If there is an obstacle, reverse immediately
        if IsObstacle_msg.obstacle:
            msg.drive.speed = -1.0
        else:
            msg.drive.speed = 0.0
        # publish the velocity command message
        self.cmd_vel_pub.publish(msg)
         
    def left_wall_callback(self, Leftsideangle):
        data = Leftsideangle.data
        # get the output from the PID controller
        self.pid_controller.update_PID(data)
        output = self.pid_controller.cmd
        # then set up a message
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = -1*math.radians(output)
	msg.drive.speed = 1.0
        # publish the command message
        self.wall_pub.publish(msg)

    def right_wall_callback(self, Rightsideangle):
        # get the output from the PID controller
        data = Rightsideangle.data
        self.pid_controller.update_PID(data)
        output = self.pid_controller.cmd
        # then set up a message
        ##msg = AckermannDriveStamped()
        #msg.header.stamp = rospy.Time.now()
        #msg.header.frame_id = frame_id
        #msg.drive.steering_angle = Rightsideangle
        # publish the command message
        #self.wall_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("Safety_Node")
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    node = SafetyNode()
    rospy.spin()
