#!/usr/bin/env python
"""lidar_to_frame: creates lidar frame/model image based on lidar data.

Subscribes:
    /scan LaserScan of points from Lidar
Publishes:
    lidar_model Image representing drivable area in front of car
"""

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from math import sin, cos, pi, isnan

IMAGE_WIDTH = rospy.get_param("image_width")
IMAGE_HEIGHT = rospy.get_param("image_height")
PIXELS_PER_M = rospy.get_param("pixels_per_m")
pub = rospy.Publisher('lidar_model', Image, queue_size=1)
bridge = CvBridge()


def gen_lidar_image(laser_scan):
    """Converts LaserScan message to Image, then publishes it."""
    # Convert points from polar to cartesian (origin at (width/2, height))
    lidar_points = laser_scan_to_cartesian(laser_scan)
    # Generate the image from the cartesian LaserScan points
    lidar_image = gen_point_image(lidar_points)
    # Publish the cv image as an imgmsg
    pub.publish(get_lidar_image_message(lidar_image))

def laser_scan_to_cartesian(laser_scan):
    """Generates a list of cartesian coordinates from LaserScan.
    Uses (width/2, height/2) as the origin."""
    lidar_points = []
    ranges = laser_scan.ranges
    # Effectively rotate the input data by 90 degrees counterclockwise
    angle = laser_scan.angle_min + pi / 2
    # Keep track of the last valid distance output by the lidar to use as a
    # substitute distance for any subsequent far or invalid lidar distances
    last_valid = 0
    # Convert points from polar to cartesian (origin at (width/2, height))
    for i in range(len(ranges)):
        # when using hokuyo_node, far points are inf.
        # when using urg_node, they're nan.
        cur = ranges[i]
        if cur != float('inf') and not isnan(cur):
            distance_to_point = ranges[i]
            # set the last valid distance on encountering a valid distance
            last_valid = distance_to_point
        else:
            # use the last valid distance on encountering lidar failure
            distance_to_point = last_valid

        lidar_points.append(
            (cos(angle) * distance_to_point * PIXELS_PER_M + (IMAGE_WIDTH / 2),
             IMAGE_HEIGHT - (sin(angle) * distance_to_point * PIXELS_PER_M))
        )
        angle += laser_scan.angle_increment
    return lidar_points

def get_lidar_image_message(lidar_image):
    """Convert an opencv image (image) to an imgmsg."""
    try:
        return bridge.cv2_to_imgmsg(lidar_image, encoding="mono8")
    except CvBridgeError:
        rospy.loginfo("Error converting lidar image to imgmsg")
        raise


def gen_point_image(points):
    """ Generate a cv image from a list of tuple (x, y) coordinates """
    # Generate image of all 0's to represent image
    image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), np.uint8)

    # Add boundary points for line drawing
    points.insert(0, (IMAGE_WIDTH//2, IMAGE_HEIGHT))
    points.append((IMAGE_WIDTH//2, IMAGE_HEIGHT))

    # Fill the drivable portion of the image with white
    cv2.fillPoly(image, [np.array(points, np.int32)], 255)
    return image

def lidar_to_frame_node():
    """Initializes lidar_to_frame node."""
    rospy.init_node('lidar_to_frame', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, gen_lidar_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__': lidar_to_frame_node()
