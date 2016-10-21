#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi

image_width = rospy.get_param("image_width")
image_height = rospy.get_param("image_height")
pixels_per_m = rospy.get_param("pixels_per_m")
image_width_m = image_width / pixels_per_m
image_height_m = image_height / pixels_per_m


lidar_publisher = rospy.Publisher('lidar_model', Image, queue_size=0)
bridge = CvBridge()


def gen_lidar_image(laser_scan):
    """ Converts a LaserScan message to a cv image, then publishes it through
        a lidar_publisher """
    #convert points from polar to cartesian (origin at (width/2, height))
    lidar_points = laser_scan_to_cartesian(laser_scan)
    #generate the image from the cartesian LaserScan points
    lidar_image = gen_point_image(lidar_points)
    #publish the cv image as an imgmsg
    lidar_publisher.publish(get_lidar_image_message(lidar_image))

def laser_scan_to_cartesian(laser_scan):
    """ Generates a list of cartesian coordinates from LaserScan range and angle
        data. Uses (width/2, height/2) as the origin. """
    lidar_points = []
    ranges = laser_scan.ranges
    #effectively rotate the input data by 90 degrees counterclockwise
    angle = laser_scan.angle_min + pi / 2
    #convert points from polar to cartesian (origin at (width/2, height))
    for i in range(len(ranges)):
        lidar_points.append(
            (cos(angle) * ranges[i] * pixels_per_m + (image_width / 2),
             image_height - (sin(angle) * ranges[i] * pixels_per_m))
        )
        angle += laser_scan.angle_increment
    return lidar_points

def get_lidar_image_message(lidar_image):
    """ Convert an opencv image (image) to an imgmsg """
    try:
        color_image = cv2.cvtColor(lidar_image, cv2.COLOR_GRAY2BGR)
        return bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
    except CvBridgeError as e:
        rospy.loginfo("Error converting lidar image to imgmsg")


def gen_point_image(points):
    """ Generate a cv image from a list of tuple (x, y) coordinates """
    # Generate image of all 0's to represent image
    image = np.zeros((image_height, image_width), np.uint8)
    image.fill(255)

    #get the points at ~pi and ~0
    min_point = points[0]
    max_point = points[len(points) - 1]

    #add boundary points for line drawing
    points.insert(0, (0,0))
    points.insert(1, (0, min_point[1]))
    points.append((image_width, max_point[1]))
    points.append((image_width, 0))

    #fill the undrivable portion of the image with black
    cv2.fillPoly(image, [np.array(points, np.int32)], 0)
    return image

def lidar_node():
    rospy.init_node('lidar_to_frame', anonymous=True)
    rospy.Subscriber('scan', LaserScan, gen_lidar_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__': lidar_node()
