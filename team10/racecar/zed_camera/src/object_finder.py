#!/usr/bin/env python
import rospy
import itertools
import threading
import time
import  math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped



class ObjectFinder:

    def __init__(self):
        self.angle=0.0
        self.angle_rad=0.0
        rospy.Subscriber('/racecar/image/data', Float64MultiArray, self.set_angle)
        self.pub=rospy.Publisher('/racecar/object/position', Float64MultiArray)#data for IBS_controller
        rospy.Subscriber("racecar/laser/objdetect", Float64MultiArray, self.find_object)
        self.angle_threshold = 9.0
        self.camera_angles = []
    
    def find_object(self,data):
        if len(self.camera_angles):
            numItems = len(data.data)/3
            lidar_angles = data.data[0:numItems]
            lidar_distances = data.data[numItems:2 * numItems]
            lidar_widths = data.data[2 * numItems: 3 * numItems]
            #print len(object_angles)
            #print 'lidar angle ', object_angles
            #print 'camera angle ', self.angle
            #print 'min diff ', [abs(angle - self.angle) for angle in object_angles]
            #print 'distances ', object_distances
            #print [object_distances[i] for i in range(numItems) if abs(object_angles[i] - self.angle) < self.angle_threshold]

            #self angles are the camera angles
            #object_angles are the lidar angles
            for angle in self.camera_angles:
                diffArray = [abs(angle - lidar_angle) for lidar_angle in lidar_angles]
                if min(diffArray) < self.angle_threshold:
                    dataToPublish = Float64MultiArray()
                    index = diffArray.index(min(diffArray))
                    dataToPublish.data = [lidar_angles[index], lidar_distances[index]]
                    self.pub.publish(dataToPublish)
                    return
 	dataToPublish = Float64MultiArray()
	dataToPublish.data = [-1, -1, -1, -1]
	self.pub.publish(dataToPublish)               
    #for i in object_angles:
    #   angle = object_angles[i]
    #   angleDiff = abs(angle - self.angle)
    '''degreeRange=110
    lengthOfData=len(data.data) - 1
    buckets=lengthOfData/float(degreeRange)
    offset=int(round(degreeRange/2))
        print 'buckets ', buckets
    index=int(round((-1*self.angle + offset) * buckets))
        #index=int((283-115)/(-6.99)*(self.angle+13.32)+283)
        #print lengthOfData
        print 'min ', min(data.data)
        print 'index ', index
        print 'index of min ', data.data.index(min(data.data))
        print 'angle ', self.angle
    indexes=data.data[int(max(index-5,0)):int(min(index+5,lengthOfData))] #error when outside range completely 
        print indexes
        distance=sum(indexes)/float(len(indexes)) 
    array=Float64MultiArray()
    array.data=[self.angle_rad,distance]
    self.pub.publish(array)'''

    def set_angle(self, data):
        self.angle_rad = data.data[0]
        self.angle = data.data[0] * 180 / math.pi #convert to degrees
        self.camera_angles = [angle * 180 / math.pi for angle in data.data]



        




if __name__ == '__main__':
    rospy.init_node('ObjectFinder', anonymous=True)
    ObjectFinder()
    rospy.spin()
