#!/usr/bin/env python
import matplotlib.pyplot as plt
import mapping.read_pgm as mapper
import sensor_update
from geometry_msgs.msg import PoseArray
import numpy as np
import os
import fcntl as F
from multiprocessing import Lock

class Plotter:
    def __init__(self):
        self.getMap()
        self.resolution = 0.05
        self.lock = Lock()
        #rospy.Subscriber("racecar/mcl/current_particles", PoseArray, self.updatePath) # from MCL
    
    def getMap(self):
        image = mapper.read_pgm("mapping/realmap.pgm", byteorder='<')
        croppedMap = mapper.hardCrop(image)
        self.map = np.flipud(np.rot90(np.array([np.array([item for item in row]) for row in croppedMap])))

    def get_path(self):
        self.lock.aquire()
        self.path = np.load("path.npy")
        self.lock.release()
        #pyx_file = os.path.join()

        

    def plot_path(self):
        print 'plotting path'
        print len(self.map)
        print len(self.path)
        for (x_m, y_m, w) in self.path[:11500]:
            x, y = sensor_update.meters_to_pixel([self.map], self.resolution, x_m, y_m)
            self.map[min(x+40, 220)][y] = 0
        plt.imshow(self.map, plt.cm.gray)
        plt.show()


if __name__ == "__main__":
    # initialize the ROS client API, giving the default node name
    # self.period = rospy.get_param('~period', self.period)

    #rospy.init_node('Plotter', anonymous=True)

    plotter = Plotter()
    while True:
        plotter.get_path()
        plotter.plot_path()  
    # enter the ROS main loop
    #rospy.spin()

