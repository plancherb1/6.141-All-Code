#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
#from ackermann_msgs.msg import AckermannDriveStamped
# ROS messages
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped

prev_speed = 5.0

def speed_find(data):
    global prev_speed
    prev_speed = data.drive.speed


def set_steer(data):
    car_ctl = AckermannDrive()
    car_msg = AckermannDriveStamped()
    steering_angle = rospy.Publisher('/vesc/racecar/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    k_distance = 6.0
    k_angle = 2.0

    if data.data[0] and data.data[3]:
        print "section 1"
        #error = k_distance * (data.data[4] - data.data[1])
        error = k_angle * (90.0 - data.data[2]) 
        print "k_distance", k_distance, "data.data[4]", data.data[4], "data_data[1]", data.data[1], "error", error

    elif data.data[0] and not data.data[3]:
        print "section 2"
        error = k_angle * (90.0 - data.data[2]) 

    elif data.data[3] and not data.data[0]:
        print "section 3"
        error = k_angle * (-90.0 - data.data[5])
    else:
        print "section 4"
        pass

    #car_ctl.acceleration = 0.0
    car_ctl.speed = 0.8
    car_ctl.steering_angle = error
    #steering_angle.publish('{drive: {steering_angle: ' + str(error) + ', speed: ' + str(prev_speed) + '}}')
    #rospy.Publisher('ackermann_cmd', AckermannDriveStamped)
    "publish pilot command message"
    car_msg.header.stamp = rospy.Time.now()
    car_msg.header.frame_id = "wall_follower"
    car_msg.drive = car_ctl
    print "right before publish, steering_angle: ", error
    steering_angle.publish(car_msg)

def wall_follower():

    rospy.init_node('wall_follower', anonymous=True)


    rospy.Subscriber("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, speed_find)
    rospy.Subscriber("racecar/laser/wall_detect", Float64MultiArray, set_steer)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass
