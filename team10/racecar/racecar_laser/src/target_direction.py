#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math
    

class OpenSpaceDetection:
    def __init__(self):
        rospy.init_node('Openspace_detection', anonymous=True)
        self.pub_target_point = rospy.Publisher('/racecar/localization/targetPoint', Point, queue_size=0)
        rospy.Subscriber("racecar/laser/scan/", LaserScan, self.find_space)
        rospy.Subscriber("scan/", LaserScan, self.find_space)
        self.lastAngle = 0
        self.biasRight = False


    def find_space(self, data):

        #list of length 440 with 0 at 55 degrees to the right and going to 55 degrees to the left (going counterclockwise)
        positive_hits = []
	data_length = len(data.ranges)
	one_sixth = data_length/6
        scan_data = data.ranges[one_sixth:5*one_sixth]

        angle_range = 180

        clusters = []
        window_size = 4
        cluster_distances = dict()
        lastCluster = set()
        distances = []
        distance_threshold = 4.
        for i in xrange(len(scan_data)-window_size):
            if sum(scan_data[i:i+window_size]) > distance_threshold * window_size:
                inCluster = False
                for cluster in clusters:
                    if i - 1 in cluster:
                        #print 'adding to old cluster ', i
                        cluster.add(i)
                        inCluster = True
                        lastCluster = cluster
                        distances += [np.mean(scan_data[i:i+window_size])]
                if not inCluster:
                    if len(lastCluster):
                        cluster_distances[tuple(sorted(lastCluster))] = distances
                    distances = []
                    #print 'creating new cluster ', i
                    newCluster = set()
                    newCluster.add(i)
                    lastCluster = newCluster
                    clusters.append(newCluster)
        cluster_distances[tuple(sorted(lastCluster))] = distances

                    
                #positive_hits += [i+5]
                #positive_hits += [(i+5, sum(scan_data[i:i+10])/10.0)] #todo -- change index to degree

        openSpaces = []
        clusterSizes = np.array([len(cluster) for cluster in clusters])
        threshold = 0
        if len(clusters) > 1:
            threshold = np.mean(clusterSizes) + np.std(clusterSizes) * .5 #might want to change this treshold, defines how big clusters have to be in order to be considered whitespace
        maxSize = 0
        maxOpenSpace = [0,0]
        maxCluster = None
        filteredClusters = dict()
        largeClusters = []
        #print 'num Clusters ', len(clusters), 'threshold ', threshold
        for cluster in clusters:
            if len(cluster) > threshold:
                rightPoint = max(cluster)
                leftPoint = min(cluster)
                openSpace_start_angle = int(round(-1 * (rightPoint - len(scan_data)/2) *  angle_range / len(scan_data)))
                openSpace_end_angle = int(round(-1 * (leftPoint - len(scan_data)/2) *  angle_range / len(scan_data)))
                filteredClusters[openSpace_start_angle] = tuple([openSpace_end_angle, sorted(cluster)])
                largeClusters += [cluster]
                if rightPoint - leftPoint > maxSize:
                    maxOpenSpace = [openSpace_start_angle, openSpace_end_angle]
                    maxCluster = cluster
                openSpaces += [[openSpace_start_angle, openSpace_end_angle]]

        print 'num Large clusters ', len(filteredClusters)
        filteredKeys = sorted(filteredClusters.keys())
        rightMost = None
        if filteredKeys[0] < -30 or filteredClusters[filteredKeys[-1]][0] < -60:
            rightMost = filteredClusters[filteredKeys[0]][1]


        angleDiffs = []
        rightAngleDiffs = []
        for cluster in largeClusters:
            rightPoint = max(cluster)
            leftPoint = min(cluster)
            openSpace_start_angle = int(round(-1 * (rightPoint - len(scan_data)/2) *  angle_range / len(scan_data)))
            openSpace_end_angle = int(round(-1 * (leftPoint - len(scan_data)/2) *  angle_range / len(scan_data)))
            delta = openSpace_end_angle - openSpace_start_angle
            '''numClusters = 5
            for i in range(numClusters):
                theta = openSpace_start_angle + delta * (i + 1) / float((numClusters + 1))
                transformed_theta = int(round(-1 * (theta - len(scan_data)/2) *  float(angle_range) / len(scan_data)))
                diff = transformed_theta - self.lastAngle
                angleDiffs += [(diff, transformed_theta)]'''
            #print 'sum ', sum(cluster)
            #print 'len ', len(cluster)
            theta = sum(cluster) / len(cluster)
            #print 'theta ', theta
            transformed_theta = int(round(-1 * (theta - len(scan_data)/2) *  float(angle_range) / len(scan_data)))
            diff = transformed_theta - self.lastAngle
            angleDiffs += [(diff, transformed_theta)]
            #print 'transformed_theta ', transformed_theta
            #print 'angle diff ', diff

        '''#print objects
        #openSpace_start_angles = [start_angle for distance, start_angle, end_angle in openSpace]
        #openSpace_end_angles = [end_angle for distance, start_angle, end_angle in openSpace]


        output_cluster = tuple(sorted(maxCluster))
        largeClusterSizes = np.array([len(cluster) for cluster in largeClusters])
        right_threshold = np.mean(largeClusterSizes)
        if rightMost is not None and len(rightMost) > right_threshold:
            output_cluster = tuple(sorted(rightMost))
        #r = sum(cluster_distances[output_cluster]) / len(output_cluster)
        #print 'sum ', sum(rightMost)
        #print 'len ', len(rightMost)
        theta = sum(output_cluster) / len(output_cluster)
        #print 'theta ', theta
        transformed_theta = int(round(-1 * (theta - len(scan_data)/2) *  270.0 / len(scan_data)))
        print 'chosen theta ', transformed_theta'''

        if not self.biasRight:
            angleDiffs = sorted(angleDiffs)
            finalDiff, chosenAngle = angleDiffs[0]

        #print 'chosenAngle, finalDiff: ', chosenAngle, ", ", finalDiff

        self.lastAngle = chosenAngle
        r = distance_threshold
        x = r * math.sin(chosenAngle * math.pi / 180)
        y = r * math.cos(chosenAngle * math.pi / 180)
        #print r, transformed_theta, theta
        print chosenAngle
        print 'new round'
        outputPoint = Point()
        outputPoint.x = x
        outputPoint.y = y
        outputPoint.z = chosenAngle

        self.pub_target_point.publish(outputPoint)


        #print len(data.ranges)


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)
    try:
        OpenSpaceDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
