#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0
flag_stop = False

def detect_stop(data):
    global flag_stop
    flag_stop = not len(data.data) == 0
    #print "changed flag_stop to ", flag_stop

def filter_command(data):

    global flag_move
    global flag_stop
    pub_movement = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
    
    newData = AckermannDriveStamped()
    newData.header = data.header

    #print "flag is ", flag_stop
    #if len(data.header.frame_id) > 0 or not flag_stop:
    newData.drive = data.drive

    pub_movement.publish(newData)


def command_filter():

    rospy.init_node('command_filter', anonymous=True)

    rospy.Subscriber("/vesc/racecar/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, filter_command)
    rospy.Subscriber("racecar/laser/objdetect", Float64MultiArray, detect_stop)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        command_filter()
    except rospy.ROSInterruptException:
        pass
