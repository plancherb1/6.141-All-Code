#!/usr/bin/env python
import rospy
import itertools
import threading
import time
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray


'''
Current status:
ImageParser is able to find sections of orange/red,
and publishes to the /racecar/image/data.  It publishes a Float64MultiArray
of the form:
[centerX, centerY, width, height, relativeAngle]
Note that this is only the details of the largest object found.
relativeAngle is between -50 degrees (in radians) and 50 degrees (in radians), with 0 being center.

If no object is found, the array will be [-1, -1, -1, -1, -1].

When running on Rodrigo's laptop with rosbag data, ImageParser is publishing
at a rate of 12-13 Hz.

Ideally we would return the distance found from the ZED camera at
the points that are found (either at the center or some sort of average),
but Rodrigo found it hard to find meaning in the ZED depth data.

ImageParser also publishes an image to /racecar/image/newIamge for 
debugging purposes.  This shows the section of the image that was found
to be of a certain color.  This image can be seen with rqt_image_view

Useful notes:
self.resolution modifies how dense our search is.  Computation decreases
*quadratically* relative to resolution.

self.object_detection_threshold specifies how many pixels have to be in a
cluster before we consdier it an object.  Note that, propabilistically speaking,
one pixel detected is equal to resolution^2 pixels in the whole image.

The color we are searching for is specified by the thresholds around line
134, as well as the if statment on line 149

#TODO
Figure out depth stuff
Make custom Message type
Publish all objects, not just largest one
Generalize to more than just orange?
'''

class ImageParser:
    def __init__(self):
        self.image_data = None
        self.depth_data = None

        #object_detection_threshold specified the number of pixels that
        #have to be identified to classify as an object.  the larger the
        #threshold, the larger the object must be in order to be recognized
        self.object_detection_threshold = 1

        #resolution -- the bigger it is, the lower the resolution,
        #but the faster the function goes.
        self.resolution = 15

        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.parse_image_data)
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.parse_depth_data)

        self.lock_image = threading.Lock()
        self.lock_depth = threading.Lock()

    def parse_image_data(self, data):
        with self.lock_image:
            self.image_data = data

    def parse_depth_data(self, data):
        with self.lock_depth:
            self.depth_data = data
        self.find_object()

    #checkAdjecency:
    #input: point/pixel, cluster
    #output: boolean indicating whether point is adjacent two cluster (directly or diagonally)
    def checkAdjacency(self, point, cluster):
        x, y = point
        return any((x + xx, y + yy) in cluster for (xx, yy) in list(itertools.permutations([-self.resolution, 0, self.resolution], 2)) + [(a, a) for a in [-self.resolution, self.resolution]])

    #reduceClusters:
    #input: list of clusters
    #output: reduced list of clusters
    #checks if any clusters border each other, and joins them if they do
    #also eliminates any clusters smaller than designated treshold
    def reduceClusters(self, clusters):
        #print 'Previous clusters: ', len(clusters)
        joined = True
        clustersCopy = clusters
        starttime = time.time()
        while joined and len(clustersCopy) > 1:
            joined = False
            for cluster1 in clustersCopy:
                for cluster2 in clustersCopy:
                    if cluster1 is not cluster2:
                        for point in cluster1:
                            if self.checkAdjacency(point, cluster2):
                                joined = True
                                cluster1.union(cluster2)
                                clustersCopy.remove(cluster2)
                                break
                if len(cluster1) <= self.object_detection_threshold:
                    clustersCopy.remove(cluster1)
        #print 'Clusters after: ', len(clustersCopy)
        #print 'Cluster reduction time: ', time.time() - starttime
        return clustersCopy

    def find_object(self):
        with self.lock_depth:
            image_data = self.image_data
            depth_data = self.depth_data

        pub_data = rospy.Publisher('/racecar/image/data', Float64MultiArray, queue_size=1)
        arrayData = Float64MultiArray()
        #print "received image"
        if image_data:
            newImage = Image()
            newImage.encoding = image_data.encoding
            newImage.header = image_data.header
            newImage.is_bigendian = image_data.is_bigendian

            newData = bytearray()
            image = image_data.data
            clusters = []
            white = chr(255) *3
            black = chr(0) * 3
            red = chr(0) + chr(0) + chr(255)
            blue = chr(255) + chr(0) * 2

            #defining color search thresholds
            redThreshold = 230
            blueThreshold = 20
            greenLowerThreshold = 50
            greenUpperThreshold = 200
            
            height = 0
            starttime = time.time()

            for h in xrange(0, image_data.height, self.resolution):
                height += 1
                for w in xrange(0, image_data.width, self.resolution):
                    index = w * 3 + image_data.width * 3 * h
                    b = ord(image[index])
                    g = ord(image[index + 1])
                    r = ord(image[index + 2])
                    #filtering for the color
                    if r > redThreshold and b < blueThreshold and g <= greenUpperThreshold and g >= greenLowerThreshold:
                        for char in white:
                            newData.append(char)
                        inCluster = False
                        for cluster in clusters:
                            if self.checkAdjacency((w, h), cluster):
                                cluster.add((w, h))
                                inCluster = True
                        if not inCluster:
                            newCluster = set()
                            newCluster.add((w, h))
                            clusters.append(newCluster)
                    else:
                        for char in black:
                            newData.append(char)

                
            newImage.height = height
            newImage.width = len(newData)/height/3
            newImage.step = len(newData)/height
            newImage.data = str(newData)


            #print 'time taken: ', time.time() - starttime
            
            clusters = self.reduceClusters(clusters)
            #print len(clusters), float(len(candidates_x))/len(newData)
            #print [len(cluster) for cluster in clusters]
            if len(clusters) > 1:
                print "multiple objects found"
                print [len(cluster) for cluster in clusters]

            if len(clusters):
                publishableClusters = [max(clusters, key=lambda x:len(x))]
                data_to_publish = []
                for cluster in publishableClusters:
                    centerX = sum(x for (x, y) in cluster)/len(cluster)
                    centerY = sum(y for (x, y) in cluster)/len(cluster)
                    width = max(cluster, key=lambda (x, y):x)[0] - min(cluster, key=lambda (x, y):x)[0]
                    height = max(cluster, key=lambda (x, y):y)[1] - min(cluster, key=lambda (x, y):y)[1]
                    relativeLocation = float(centerX)/image_data.width - 0.5
                    relativeAngle = relativeLocation * 100 * math.pi / 180
                    data_to_publish += [centerX, centerY, width, height, relativeAngle]
                    #print 'center ', centerX, centerY, len(candidates_x)
                    #print 'distance ', depth1, depth2
                    #print 'dim ', height, width
                    '''depth_index = centerX * 2 + image_data.width * 2 * centerY
                    depth1 = ord(depth_data.data[depth_index])
                    depth2 = ord(depth_data.data[depth_index + 1])'''

                arrayData.data = data_to_publish
            else:

                arrayData.data = [-1, -1, -1, -1, -1]
                print 'no cone found'
            pub_data.publish(arrayData)
            #print "publishing"
            pub_img = rospy.Publisher('/racecar/image/newImage', Image, queue_size=1)
            pub_img.publish(newImage)
        else:
            arrayData.data = [-1, -1, -1, -1]
            print 'image not ready'


if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)

    ImageParser()

    rospy.spin()
