import cv2
import numpy as np

import time

image_height_px = 800
image_width_px = 800
image_height_m = 8
image_width_m = 8
pixels_per_m = 100


# lidarPoints is a list of tuples. i.e. [(x0, y0), (x1, y1), ...]
def genLidarImage(lidarPoints):
    # Generate matrix of all 0's to represent image
    matrix = np.zeros((image_height_px, image_width_px), np.uint8)
    matrix.fill(255)
 
    # Remap points to from lidar coordinates to image coordinates
    remappedPoints = []
    for point in lidarPoints:
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

    cv2.fillConvexPoly(matrix, np.array(polyPoints, np.int32), 0)
    cv2.imshow("yo", matrix)
    cv2.waitKey(0)
    cv2.destroyAllWindows()    


genLidarImage([(1, 2.4), (2, 3.3), (2.7, 2.7), (3.5, 3.6)])
