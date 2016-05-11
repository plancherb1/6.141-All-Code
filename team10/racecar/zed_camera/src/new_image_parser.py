#!/usr/bin/env python
import rospy
import itertools
import threading
import time
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
from pylab import array, plot, show, axis, arange, figure, uint8 



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
        self.threshold = 10

        self.bridge = CvBridge()

        rospy.Subscriber("rgb/image_rect", Image, self.parse_image_data)
        rospy.Subscriber("depth/image_rect", Image, self.parse_depth_data)


        self.pub_data = rospy.Publisher('/racecar/image/data', Float64MultiArray, queue_size=1)
        self.pub_img = rospy.Publisher('/racecar/image/newImage', Image, queue_size=1)

        self.debug_img = rospy.Publisher('/racecar/image/debug', Image, queue_size=1)

        self.lock_image = threading.Lock()
        self.lock_depth = threading.Lock()

    def parse_image_data(self, data):
        with self.lock_image:
            self.image_data = data
        self.find_object()

    def parse_depth_data(self, data):
        with self.lock_depth:
            self.depth_data = data
        self.find_object()

    def find_object(self):
        with self.lock_depth:
            image_data = self.image_data
            depth_data = self.depth_data

        arrayData = Float64MultiArray()
        arrayData.data = [-1, -1, -1, -1]
        #print "received image"
        if image_data:

            im = self.bridge.imgmsg_to_cv2(image_data)
            height, width, channels = im.shape
            '''h, w, c = im.shape
            filteredIm = np.zeros((h, w, 3), np.uint8)
            #filteredIm[:,0:0.5*w] = (255,0,0)      # (B, G, R)
            #filteredIm[:,0.5*w:w] = (0,255,0)


            for i in range(h):
                row = im[i]
                for j in range(w):
                    [b, g, r] = row[j]
                    if g > r and g > b:
                        filteredIm[i][j] = [255, 255, 255]
                    else:
                        filteredIm[i][j] = [0, 0, 0]
            
            

            maxIntensity = 255.0 # depends on dtype of image data
            x = arange(maxIntensity) 

            # Parameters for manipulating image data
            phi = 1
            theta = 1

            # Increase intensity such that
            # dark pixels become much brighter, 
            # bright pixels become slightly bright
            newImage0 = (maxIntensity/phi)*(im/(maxIntensity/theta))**0.5
            newImage0 = array(newImage0,dtype=uint8)


            try:
                self.debug_img.publish(\
                        self.bridge.cv2_to_imgmsg(filteredIm, "bgr8"))
            except CvBridgeError as e:
                print(e)'''


            hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

            #COLOR_MIN = np.array([70, 60, 130],np.uint8)
            #COLOR_MAX = np.array([100, 255, 255],np.uint8)
            COLOR_MIN = np.array([60, 31, 102],np.uint8)
            COLOR_MAX = np.array([80, 200, 255],np.uint8)
            frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
            imgray = frame_threshed
            ret,thresh = cv2.threshold(frame_threshed,5,255,0)
            contours, hierarchy = cv2.findContours(\
                    thresh,cv2.RETR_LIST ,cv2.CHAIN_APPROX_NONE)

            # Find the index of the largest contour
            contourAreas = [(cv2.contourArea(c), c) for c in contours]
            try:
                contourAreas.sort()
            except:
                pass
            largeContours = [c for area, c in contourAreas if area > self.threshold]
            #print [cv2.contourArea(c) for c in largeContours]
            #print sorted(areas)
            #max_index = np.argmax(areas)
            #cnt = contours[max_index]
            pubArray = []
            preArray = []
            for cnt in largeContours:
                #print cnt
            # plot box around contour
                x,y,w,h = cv2.boundingRect(cnt)
                centerX = round(x + w/2.)
                centerY = round(y + w/2.)
                [b, g, r] = im[centerY][centerX]
                unevenColors = bool(min([r,g,b]) + 8 < max([r,g,b]))
                if w > h * 2 and not unevenColors and cv2.contourArea(cnt) < 13000:
                    '''if g > b:
                        print 'more green!'
                    else:
                        print 'MORE BLUE!'''
                    if min([r,g,b]) + 8 < max([r,g,b]):
                        print 'uneven colors'
                    #print b, g, r
                    #print centerY, height
                    #print cv2.contourArea(cnt)
                    cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)

                    relativeLocation = float(x + w/2.)/im.shape[1] - 0.5
                    relativeAngle = relativeLocation * 100 * math.pi / 180.
                    preArray += [(cv2.contourArea(cnt), relativeAngle)]
                #areaArray.append(cv2.contourArea(cnt))
            preArray.sort()
            pubArray = [angle for area, angle in preArray]
            if len(pubArray):
                arrayData.data = pubArray

            '''try:
                self.pub_img.publish(\
                        self.bridge.cv2_to_imgmsg(im, "bgr8"))
            except CvBridgeError as e:
                print(e)'''


           
           # self.pub_data.publish(arrayData)
            #print "publishing"
            
        else:
            #arrayData.data = [-1, -1, -1, -1]
            print 'image not ready'
        (arrayData.data).reverse()
        self.pub_data.publish(arrayData)
       


if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)

    ImageParser()

    rospy.spin()
