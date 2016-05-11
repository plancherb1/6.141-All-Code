#!/usr/bin/python
#
import rospy
from racecar_nav.msg import IsObstacle
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode:
    def __init__(self):
        self.safetydistance = 0.5
        self.linear_scale = 2.0
        self.angular_scale = 2.0
        # subscribe to the IsObstacle topic
        rospy.Subscriber("IsObstacle", IsObstacle, self.safety_callback)
        # advertise that we'll publish on the cmd_vel topic for ackermann_mux_cmd
        self.cmd_vel_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)

    def safety_callback(self, IsObstacle_msg):
        global frame_id
        global wheelbase
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.drive.steering_angle = 0.0
        # If there is an obstacle, reverse immediately
        if IsObstacle_msg.obstacle:
            msg.drive.speed = 0.0
        else:
            msg.drive.speed = 0.0
        # publish the velocity command message
        self.cmd_vel_pub.publish(msg)
         


if __name__ == '__main__':
    rospy.init_node("Safety_Node")
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    frame_id = rospy.get_param('~frame_id', 'odom')
    node = SafetyNode()
    rospy.spin()
