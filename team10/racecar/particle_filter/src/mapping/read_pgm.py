import re
import numpy
import matplotlib.pyplot as plt
import time
import math

def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
	

    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))

'''
    This file creates maps of the distance to the closest walls at the angles specified in outputAngles.
    The output is structured as a three tiered array, in [theta][x][y] order.  So to access the
    distance map at angle 90, one would access output[angles.index(90)].

    Currently it is set up as if the map were a coordinate plane, 0 is the positive x axis, and
    angles increase counterclockwise.  This could easily be changed if needed.

'''

master_dist_increment = 0.05
anglesToCoordinates = {0: (1, 0), 30: (2, -1), 45: (1,-1), 60: (1, -2), 
                        90: (0, -1), 120: (-1, -2), 135: (-1, -1), 150: (-2, -1),
                        180: (-1, 0), 210: (-2, 1), 225:(-1, 1), 240:(-1, 2),
                        270: (0, 1), 300: (1, 2), 315: (1, 1), 330:(2, 1)}


angles = [0, 30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 240, 270, 300, 315, 330]
dist_increments = {angle: master_dist_increment * [math.sqrt(a * a + b * b) for a, b in [anglesToCoordinates[angle]]][0] for angle in angles}
white = 254
grey = 205
black = 0
def next_pixel(pixel, angle):
    return [a + b for a, b in zip(pixel, anglesToCoordinates[angle][::-1])]

def start_pixel(i, angle, vertical, shift, height, width):
    shift = int(shift)
    if 0 <= angle < 90:
        #angle in quadrant I
        if vertical:
            return [i, shift]
        else:
            return [height - 1 - shift, i]
    if 90 <= angle < 180:
        #angle in quadrant II
        if vertical:
            return [i, width - 1 - shift]
        else:
            return [height - 1 - shift, i]
    if 180 <= angle < 270:
        #angle in quadrant III
        if vertical:
            return [i, width - 1 - shift]
        else:
            return [shift, i]
    if 270 <= angle < 360:
        #angle in quadrant IV
        if vertical:
            return [i, shift]
        else:
            return [shift, i]
    
def new_parse_image(image):
    height = len(image)
    width = len(image[0])
    area = height * width
    distanceLayer = numpy.array([0.0 for column in range(width)])
    rowLayer = numpy.array([distanceLayer for row in range(height)])
    output = numpy.array([rowLayer for angle in range(len(angles))])
    visitedPoses = {angle: set() for angle in angles}
    
    #output = {angle : numpy.copy(rowLayer) for angle in angles}
    def start_pixel(i, angle, vertical, shift):
        shift = int(shift)
        if 0 <= angle < 90:
            #angle in quadrant I
            if vertical:
                return [i, shift]
            else:
                return [height - 1 - shift, i]
        if 90 <= angle < 180:
            #angle in quadrant II
            if vertical:
                return [i, width - 1 - shift]
            else:
                return [height - 1 - shift, i]
        if 180 <= angle < 270:
            #angle in quadrant III
            if vertical:
                return [i, width - 1 - shift]
            else:
                return [shift, i]
        if 270 <= angle < 360:
            #angle in quadrant IV
            if vertical:
                return [i, shift]
            else:
                return [shift, i]
    
    def checkPixel(y, x, dist, angle):
        if image[y][x] == black:
            newDist = 0
        if image[y][x] == grey:
            newDist = dist
        if image[y][x] == white:
            newAngle = (angle + 180) % 360
            z = angles.index(newAngle)
            output[z][y][x] = round(dist, 5)
            newDist = dist + dist_increments[angle]
        return newDist

    def runThroughImage(vertical, shift, angle):
        if vertical:
            loopMax = height
        else:
            loopMax = width
        for i in range(loopMax):
            dist = 0
            y, x = start_pixel(i, angle, vertical, shift)
            while y < height and x < width and y >= 0 and x >= 0:
                if (y, x) in visitedPoses[angle]:
                    break
                dist = checkPixel(y, x, dist, angle)
                visitedPoses[angle].add(tuple([y, x]))
                newY, newX = next_pixel([y, x], angle)
                if abs(newY - y) == 2 and newY < height and newY >= 0 and newX < width and newX >= 0:
                    diff = (newY - y) / 2
                    if checkPixel(y + diff, x, dist, angle) == 0 or checkPixel(y + diff, newX, dist, angle) == 0:
                        dist = 0
                if abs(newX - x) == 2 and newX < width and newX >= 0 and newY < height and newY >= 0:
                    diff = (newX - x) / 2
                    if checkPixel(y, x + diff, dist, angle) == 0 or checkPixel(newY, x + diff, dist, angle) == 0:
                        dist = 0
                x, y = newX, newY
                
    for angle in angles:
        startTime = time.time()
        print angle
        print "dist increment: ", dist_increments[angle]
        runThroughImage(True, False, angle)
        yCount = len(visitedPoses[angle])
        runThroughImage(False, False, angle)
        if float(len(visitedPoses[angle]))/area < 1.0:
            runThroughImage(True, True, angle)
        if float(len(visitedPoses[angle]))/area < 1.0:
            runThroughImage(False, True, angle)
        count = len(visitedPoses[angle])
        xCount = count - yCount
        print 'time: ', time.time() - startTime
        print 'total count: ', float(count)/area
        print 'xCount: ', xCount/float(count)
        print 'yCount: ', yCount/float(count)
    return output
                    
            
            
def cropVertically(image):
    firstRow = 0
    lastRow = len(image)
    firstRowSet = False
    i = len(image) / 2
    for i in range(len(image)):
        if i % 100 == 0:
            print i
        count = len([pix for pix in image[i] if pix != 205])
        if not firstRowSet and count:
            firstRow = i
            firstRowSet = True
        if not count and firstRowSet:
            lastRow = i
            break
    print firstRow, lastRow
    return image[firstRow:lastRow]
    

def hardVertical(image):
    return image[1793:2105]

def hardHorizontal(image):
    return image[1828:2049]

def cropImage(image):
    croppedVertical = cropVertically(image)
    rotatedCroppedVertical = numpy.rot90(croppedVertical)
    rotatedCroppedFinal = cropVertically(rotatedCroppedVertical)
    return numpy.rot90(rotatedCroppedFinal, 3)
    
def hardCrop(image):
    croppedVertical = hardVertical(image)
    rotatedCroppedVertical = numpy.rot90(croppedVertical)
    rotatedCroppedFinal = hardHorizontal(rotatedCroppedVertical)
    return numpy.rot90(rotatedCroppedFinal, 3)
    

if __name__ == "__main__":
    image = read_pgm("realmap.pgm", byteorder='<')

    image = hardCrop(image)
    #pyplot.imshow(image, pyplot.cm.gray)
    #pyplot.show()
    #output = new_parse_image(image)
    #for i in range(len(angles)):
    #    pyplot.imshow(numpy.fliplr(output[i]), pyplot.cm.gray)
    #    pyplot.show()
    #print len(output[0])
    #print len(output)
    #plt.plot([output[0][160][y] for y in range(len(output[0]))])
    #plt.show()
    
    """data = output
    print data.shape
    with file('map.txt', 'w') as outfile:
        # I'm writing a header here just for the sake of readability
        # Any line starting with "#" will be ignored by numpy.loadtxt
        outfile.write('# Array shape: {0}\n'.format(data.shape))

        # Iterating through a ndimensional array produces slices along
        # the last axis. This is equivalent to data[i,:,:] in this case
        for data_slice in data:
            # The formatting string indicates that I'm writing out
            # the values in left-justified columns 7 characters in width
            # with 2 decimal places.  
            numpy.savetxt(outfile, data_slice, fmt='%-7.2f')

            # Writing out a break to indicate different slices...
            outfile.write('# New slice\n')

    rawmap = numpy.loadtxt('map.txt')
    rawmap = rawmap.reshape((16, 312, 221))
    print numpy.all(data == rawmap)
    print numpy.max(data-rawmap)
    print numpy.min(data-rawmap)
    print numpy.mean(data - rawmap)"""
    white_spots = numpy.transpose((image == 254).nonzero())
    print len(white_spots)/float(len(image))
    '''
    #code for testing things on a small map
    rows = [0 for i in range(10)]
    test = [rows for i in range(10)]
    test = numpy.array(test)
    height = len(test)
    width = len(test[0])
    angle = 300
    def runThroughImage(vertical, shift):
        if vertical:
            loopMax = height
        else:
            loopMax = width
        for i in range(loopMax):
            y, x = start_pixel(i, angle, vertical, shift, height, width)
            while y < height and x < width and y >= 0 and x >= 0:
                test[y][x] = 1
                y, x = next_pixel([y, x], angle)'''


