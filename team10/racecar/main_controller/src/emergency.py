#!/usr/bin/python
#
import rospy
from threading import Lock
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class EmergencyNode:
    def __init__(self):
	# subscribe to the laser data
        rospy.Subscriber("/scan", LaserScan, self.emergency_callback)
	# subscribe to the move commands and execute them as long as not in an emergency
	rospy.Subscriber("/racecar/move_commands",AckermannDriveStamped, self.move_callback)
        # advertise that we'll publish on the racecar/emergency_stop topic for notifications
        self.emergency_pub = rospy.Publisher("racecar/emergency_stop", Bool, queue_size = 1)
 	# advertise that we'll publish on the cmd_vel topic for ackermann_mux_cmd
        self.cmd_vel_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)

	# global variables
        self.safety_distance = 0.5
	self.emergency_time_delta = rospy.Duration.from_sec(0.5)
	self.emergency_lock = Lock()
	self.last_move_command = None

	# stop command
	msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
	msg.drive.steering_angle = 0.0
	msg.drive.speed = 0.0
	self.stop_command = msg

	# reverse command
	msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
	msg.drive.steering_angle = 0.0
	msg.drive.speed = -1.0
	self.reverse_command = msg

    # See if obstacle is within cone of +/-30 degrees
    def emergency_callback(self, LaserScan_msg):
	if self.emergency_lock.acquire(False):
	    # Laser index 0:1080, using 420:660 for +/- 30 degrees
	    cone = list(LaserScan_msg.ranges[420:660])
            print min(cone)
	    if min(cone) < self.safety_distance:
		dangerIndicies = [(cone.index(i) + 420) for i in cone if i <= self.safety_distance]
		#There might be dirt on the laser, so check if over threshold size
		if len(dangerIndicies) > 5:
		    #publish emergency and stop the car for time delta
		    self.emergency_pub.publish(1)
		    self.cmd_vel_pub.publish(self.reverse_command)
		    rospy.Timer(self.emergency_time_delta,self.emergency_over,1)
		else:
		    self.emergency_pub.publish(0)
		    self.emergency_lock.release()
	    else:
	        self.emergency_lock.release()

    # release the lock and allow move commands because out of danger
    def emergency_over(self,event):
	self.emergency_pub.publish(0)
	if not (self.last_move_command == None):
	    self.cmd_vel_pub.publish(self.last_move_command)
	else:
	    self.cmd_vel_pub.publish(self.stop_command)
	self.emergency_lock.release()
    
    # Continue to publish move commands unless in an emergency
    def move_callback(self, move_command):
	if self.emergency_lock.locked():
	    self.last_move_command = move_command
	else:
	    self.cmd_vel_pub.publish(move_command)

if __name__ == '__main__':
    rospy.init_node("EmergencyNode")
    node = EmergencyNode()
    rospy.spin()
