#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
import numpy as np
#from racecar_laser.msg import Object
    

def distCheck(distances, i, scan_data):
    if not len(distances):
        return True
    dist = np.mean(scan_data)
    if dist > np.mean(distances) * 1.1 or dist < np.mean(distances) * 0.9:
        return True
    else:
        return False

def find_object(data):

    #list of length 440 with 0 at 55 degrees to the right and going to 55 degrees to the left (going counterclockwise)
    positive_hits = []
    scan_data = data.data

    angle_range = 110

    clusters = []
    window_size = 4
    for i in xrange(len(scan_data)-window_size):
        distances = []
        if sum(scan_data[i:i+window_size]) < 5 * window_size:
            inCluster = False
            for cluster in clusters:
                if i - 1 in cluster and distCheck(distances, i, scan_data[i:i+window_size]):
                    #print 'adding to old cluster ', i
                    cluster.add(i)
                    inCluster = True
                    distances += np.mean(scan_data[i:i+window_size])
            if not inCluster:
                distances = []
                #print 'creating new cluster ', i
                newCluster = set()
                newCluster.add(i)
                clusters.append(newCluster)
                
            #positive_hits += [i+5]
            #positive_hits += [(i+5, sum(scan_data[i:i+10])/10.0)] #todo -- change index to degree

    objects = []
    #print 'clusters: ', clusters
    for cluster in clusters:
        #detected_object = Object()
        #centerPoint = int(round(sum(cluster)/len(cluster)))
        rightPoint = max(cluster)
        leftPoint = min(cluster)
        object_start_angle = int(round(-1 * (rightPoint - len(data.data)/2) *  angle_range / len(data.data)))
        object_end_angle = int(round(-1 * (leftPoint - len(data.data)/2) *  angle_range / len(data.data)))
        #object_angle = -1 * (centerPoint - len(data.data)/2) *  angle_range / len(data.data)
        #object_width = max(cluster) - min(cluster)
        object_distance = sum(scan_data[i] for i in cluster)/len(cluster)

        objects += [(object_distance, object_start_angle, object_end_angle)]

    objects.sort()
    #print objects
    object_start_angles = [start_angle for distance, start_angle, end_angle in objects]
    object_end_angles = [end_angle for distance, start_angle, end_angle in objects]
    object_distances = [distance for distance, start_angle, end_angle in objects]

    
    #print 'object_angles: ', object_angles


    pub_detected_objects = rospy.Publisher('/racecar/laser/objdetect', Float64MultiArray, queue_size=1)
    #print positive_hits
    '''objects = Object()
    objects.angles = object_angles
    objects.widths = object_widths
    objects.distances = object_distances'''
    objectsToPublish = Float64MultiArray()
    objectsToPublish.data = object_start_angles + object_end_angles + object_distances
    pub_detected_objects.publish(objectsToPublish)
    
    #print len(data.ranges)

def object_detection():

    rospy.init_node('object_detection', anonymous=True)

    rospy.Subscriber("racecar/laser/scan/center", Float64MultiArray, find_object)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        pass
