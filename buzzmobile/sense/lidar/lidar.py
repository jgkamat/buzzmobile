import cv2
import numpy as np
import constants

import time

from cv_bridge import CvBridge, CvBridgeError

image_height_px = 800
image_width_px = 800
image_height_m = 8
image_width_m = 8
pixels_per_m = 100


lidar_publisher = rospy.Publisher('lidar_model', Image, queue_size=0)

bridge = CvBridge()

def gen_lidar_image(laser_scan):
    """ Converts a LaserScan message to a cv image, then publishes it through 
        a lidar_publisher """
    lidar_points = []
    ranges = laser_scan.ranges
    angle = laser_scan.angle_min
    #convert points from polar to cartesian (LaserScan origin is at (width/2, height)
    for i in range(len(ranges)):
        lidar_points.append((cos(angle) * ranges[i] + constants.image_width / 2, constants.image_height - sin(angle) * ranges[i]))
        angle += laser_scan.angle_increment
    #generate the image from the cartesian LaserScan points
    matrix = gen_point_image(lidar_points)
    #publish the cv image as an imgmsg
    lidar_publisher.publish(get_lidar_image_message(matrix))
    
def get_lidar_image_message(matrix):
    """ Convert an opencv image (matrix) to an imgmsg """
    try:
        return bridge.cv2_to_imgmsg(result)
    except CvBridgeError as e:
        rospy.loginfo("Error converting lidar image to imgmsg")

# points is a list of tuples. i.e. [(x0, y0), (x1, y1), ...]
def gen_point_image(points):
    # Generate matrix of all 0's to represent image
    matrix = np.zeros((image_height_px, image_width_px), np.uint8)
    matrix.fill(255)
 
    # Remap points to from lidar coordinates to image coordinates
    remappedPoints = []
    for point in points:
	x = point[0] * pixels_per_m
        y = point[1] * pixels_per_m
	newx = x + (image_width_px / 2)
        newy = image_height_px - y
        remappedPoints.append((newx, newy))

    remappedPoints = sorted(remappedPoints, key=lambda xy: xy[0])

    minPoint = min(remappedPoints)
    maxPoint = max(remappedPoints)
    
    remappedPoints.insert(0, (0,0))
    remappedPoints.insert(1, (0, minPoint[1]))
    remappedPoints.append((image_width_px, maxPoint[1]))
    remappedPoints.append((image_width_px, 0))

    #remappedPoints = sorted(remappedPoints, key=lambda xy: xy[0])

    polyPoints = []
    for pt in remappedPoints:
        polyPoints.append([pt[0], pt[1]])
    
    print polyPoints

    #cv2.fillConvexPoly(matrix, np.array(polyPoints, np.int32), 0)
    cv2.fillPoly(matrix, [np.array(polyPoints, np.int32)], 0)
    '''cv2.imshow("yo", matrix)
    cv2.waitKey(0)
    cv2.destroyAllWindows()'''
    return matrix


#genLidarImage([(1, 2.4), (2, 3.3), (1.5, 9), (3.5, 3.6)])

def lidar_node():
    rospy.init_node('lidar_node', anonymous=True)
    rospy.Subscribe('scan', LaserScan, gen_lidar_image, queue_size=0)
    rospy.spin()

if __name__ == '__main__': lidar_node()







