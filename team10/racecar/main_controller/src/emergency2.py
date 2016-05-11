#!/usr/bin/python
#
import rospy
from threading import Lock
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class EmergencyNode2:
    def __init__(self):
	# subscribe to the laser data
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
	# subscribe to the move commands and execute them as long as not in an emergency
	rospy.Subscriber("/racecar/move_commands",AckermannDriveStamped, self.move_callback)
	# advertise that we'll publish on the racecar/emergency_stop topic for notifications
        self.emergency_pub = rospy.Publisher("racecar/emergency_stop", Bool, queue_size = 0)
	# advertise that we'll publish on the cmd_vel topic for ackermann_mux_cmd for normal
	# moves and for emergency settings
        self.safety_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
	self.control_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=0)

	# global variables
	self.num_indicies_for_danger = 5
        self.safety_distance = 0.5
	self.emergency_duration = 0.2
	self.emergency_steps = 10
	self.emergency_time_delta = rospy.Duration.from_sec(self.emergency_duration/self.emergency_steps)
	self.emergency_lock = Lock()
	self.curr_emergency_steps = 0
	self.previous_velocity = None
	self.max_velocity_change = 0.1
	self.max_velocity = 2.2

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
	msg.drive.speed = -1
	self.reverse_command = msg

    # See if obstacle is within cone of +/-30 degrees
    def laser_callback(self, LaserScan_msg):
	if self.emergency_lock.acquire(False):
	    # Laser index 0:1080, using 420:660 for +/- 30 degrees and 480:600 for +- 15 degrees
	    shift = 17 * 4 # 13 degrees off
	    cone = list(LaserScan_msg.ranges[480+shift:600+shift])
	    if min(cone) < self.safety_distance:
		dangerIndicies = [(cone.index(i) + 420) for i in cone if i <= self.safety_distance]
		#There might be dirt on the laser, so check if over threshold size
		if len(dangerIndicies) > self.num_indicies_for_danger:
		    #publish emergency and stop the car for time delta
		    self.emergency_pub.publish(1)
		    self.emergency_time_step(None)
		else:
		    self.emergency_pub.publish(0)
		    self.emergency_lock.release()
	    else:
	        self.emergency_lock.release()

    # keep moving back for specified time steps
    def emergency_time_step(self,event):
	self.reverse_command.header.stamp = rospy.Time.now()
	self.safety_pub.publish(self.reverse_command)
	# done moving backward
	if self.curr_emergency_steps == self.emergency_steps:
	    self.curr_emergency_steps = 0
	    self.out_of_danger()
	# keep moving backward
	else:
	    self.curr_emergency_steps += 1
	    rospy.Timer(self.emergency_time_delta,self.emergency_time_step,1)

    # out of danger so restart the move commands after stopping going backward
    def out_of_danger(self):
	#self.safety_pub.publish(self.stop_command)
	self.emergency_pub.publish(0)
	self.stop_command.header.stamp = rospy.Time.now()
	self.emergency_lock.release()
	self.move_callback(self.stop_command)

    # Continue to publish move commands unless in an emergency
    def move_callback(self, move_command):
	if not self.emergency_lock.locked():
	    # move forward but makesure your velocity isn't too high or your acceleration isn't too big
	    if not (self.previous_velocity == None):
	        move_command.drive.speed = min(self.max_velocity,move_command.drive.speed,self.previous_velocity + self.max_velocity_change)
	    self.previous_velocity = move_command.drive.speed
	    # save the angle for going backward if we get outselves stuck
	    #self.reverse_command.drive.steering_angle = move_command.drive.steering_angle
	    # then send the command
	    self.control_pub.publish(move_command)

if __name__ == '__main__':
    rospy.init_node("EmergencyNode2")
    node = EmergencyNode2()
    rospy.spin()
