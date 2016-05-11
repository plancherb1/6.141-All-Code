#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


def forward_laser(data):
    pub_laser = rospy.Publisher('/scan', LaserScan, queue_size=1)
    pub_laser.publish(data)


def forward_teleop(data):
    pub_teleop = rospy.Publisher('/vesc/racecar/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    pub_teleop.publish(data)

def racecar_topic_forward():

    rospy.init_node('rasecar_topic_forward', anonymous=True)

    rospy.Subscriber("racecar/laser/scan", LaserScan, forward_laser)
    rospy.Subscriber("racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, forward_teleop)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        racecar_topic_forward()
    except rospy.ROSInterruptException:
        pass
