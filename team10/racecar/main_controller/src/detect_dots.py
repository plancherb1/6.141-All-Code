#!/usr/bin/env python
import rospy
import itertools
import threading
import time
import math
import cv2
import numpy as np
<<<<<<< HEAD
        #use http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

class Detecter:
    def __init__(self):
        rospy.init_node('circle_detector', anonymous=True)
        rospy.Subscriber('/camera/rgb/image_rect_color',Image,self.find_dots)
    
    def find_dots(self,data):
        print data.encoding
        array=np.array(list(data.data), dtype=np.float32)
        img = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
        img = cv2.medianBlur(img,5)
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
=======
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image #sensor_msgs/LaserScan
from std_msgs.msg import Float64MultiArray
import cv2.cv as cv
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
        self.threshold = 1500

        self.bridge = CvBridge()

        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.parse_image_data)
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.parse_depth_data)


        self.pub_data = rospy.Publisher('/racecar/image/data', Float64MultiArray, queue_size=1)
        self.pub_img = rospy.Publisher('/racecar/image/newImage', Image, queue_size=1)

        self.lock_image = threading.Lock()
        self.lock_depth = threading.Lock()

    def parse_image_data(self, data):
        with self.lock_image:
            self.image_data = data

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
        if image_data:
            im = self.bridge.imgmsg_to_cv2(image_data)
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            im = cv2.medianBlur(im,5)
            circles = cv2.HoughCircles(im,cv.CV_HOUGH_GRADIENT,1,200,param1=50,param2=50,minRadius=10,maxRadius=0)
            print "image processed"
            circles = np.uint16(np.around(circles))
            print circles
            for i in circles[0,:]:
>>>>>>> 6339987b0488ac291cdf4f89a9172a0cb0b3f4cd
               # draw the outer circle
               cv2.circle(im,(i[0],i[1]),i[2],(0,255,0),2)
              # draw the center of the circle
               cv2.circle(im,(i[0],i[1]),2,(0,0,255),3)
 
            cv2.imshow('detected circles',im)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
           

        


if __name__ == '__main__':
    rospy.init_node('detect_dots', anonymous=True)

    ImageParser()

    rospy.spin()

