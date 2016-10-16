#!/usr/bin/env python

import cv2
import numpy as np
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, atan2
import rospy



#import buzzmobile.include.constants as c

class Dum(object):
    def __init__(self): pass

c = Dum()
c.image_width = 800
c.image_height = 800
c.pixels_per_m = 100
c.image_width_m = c.image_width / c.pixels_per_m
c.image_height_m = c.image_height / c.pixels_per_m



lidar_publisher = rospy.Publisher('lidar_model', Image, queue_size=0)
bridge = CvBridge()

def gen_lidar_image(laser_scan):
    """ Converts a LaserScan message to a cv image, then publishes it through 
        a lidar_publisher """
    lidar_points = []
    ranges = laser_scan.ranges
    angle = laser_scan.angle_min + pi / 2
    #convert points from polar to cartesian (LaserScan origin is at (width/2, height)
    for i in range(len(ranges)):
        lidar_points.append((cos(angle) * ranges[i], sin(angle) * ranges[i]))
        angle += laser_scan.angle_increment
    #print lidar_points
    #generate the image from the cartesian LaserScan points
    matrix = gen_point_image(lidar_points)
    #publish the cv image as an imgmsg
    lidar_publisher.publish(get_lidar_image_message(matrix))
    #rospy.loginfo('publishing')
    
def get_lidar_image_message(matrix):
    """ Convert an opencv image (matrix) to an imgmsg """
    try:
        colorMatrix = cv2.cvtColor(matrix, cv2.COLOR_GRAY2BGR)
        return bridge.cv2_to_imgmsg(colorMatrix, encoding="bgr8")
    except CvBridgeError as e:
        rospy.loginfo("Error converting lidar image to imgmsg")

# points is a list of tuples. i.e. [(x0, y0), (x1, y1), ...]
def gen_point_image(points):
    # Generate matrix of all 0's to represent image
    matrix = np.zeros((c.image_height, c.image_width), np.uint8)
    matrix.fill(255)
 
    # Remap points to from lidar coordinates to image coordinates
    remappedPoints = []
    for point in points:
        x = point[0] * c.pixels_per_m + (c.image_width / 2)
        y = c.image_height - (point[1] * c.pixels_per_m)
        remappedPoints.append((x, y))

    minPoint = remappedPoints[0]
    maxPoint = remappedPoints[len(remappedPoints) - 1]
    
    remappedPoints.insert(0, (0,0))
    remappedPoints.insert(1, (0, minPoint[1]))
    remappedPoints.append((c.image_width, maxPoint[1]))
    remappedPoints.append((c.image_width, 0))

    #remappedPoints = sorted(remappedPoints, key=lambda xy: atan2(xy[0], xy[1]))
    
    polyPoints = []
    for pt in remappedPoints:
        polyPoints.append([pt[0], pt[1]])
    
    #print polyPoints

    #cv2.fillConvexPoly(matrix, np.array(polyPoints, np.int32), 0)
    cv2.fillPoly(matrix, [np.array(polyPoints, np.int32)], 0)
    #cv2.imshow("yo", matrix)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return matrix


#gen_point_image([(1, 2.4), (2, 3.3), (1.5, 9), (3.5, 3.6)])

def lidar_node():
    rospy.init_node('lidar_node', anonymous=True)
    rospy.Subscriber('scan', LaserScan, gen_lidar_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__': lidar_node()







