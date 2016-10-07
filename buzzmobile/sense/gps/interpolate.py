import numpy as np
import math
import cv2
import maps
import random
import rospy

def interpolate(points, sigma_x, sigma_y, height=500, width=500):
    """
    Takes a list of points, connects them with a line,
    and Gaussian blurs the line.
    ----------------------------------------------------------------------------
    points: list of tuples, i.e. (x, y) points that will be plotted on the image
    sigma_x, sigma_y: gaussian kernel paramters (higher = more blurry)
    height, width: dimensions of the output image in pixels
    ----------------------------------------------------------------------------
    """
    output = np.zeros((height, width), np.uint8)
    x_vals = [x for (x, y) in points]
    y_vals = [y for (x, y) in points]
    left = min(x_vals)
    bottom = max(y_vals)
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(output, pt1, pt2, [255, 255, 255], 4)
    output = cv2.GaussianBlur(output, (sigma_x, sigma_y), 0)
    return output
    #pub = rospy.Publisher('gps_image', Image, queue_size = 10)
    #rospy.init_node('gps', anonymous=True)

def nearest_point_index(location, points_array):
    """
    Returns the index of the point in the given numpy array of points
    that is closest to the specified location.
    """
    deltas = points_array - location
    distances = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(distances)


def close_points(points, max_distance):
    """
    Returns a slice of the input list of points such that the slice ends at
    the given maximum distance from the first point in the list of points.
    """
    start = points[0]
    x, y = start
    final_index = 0
    for i in range(len(points)):
        distance = maps.haversine(x, y, points[i][0], points[i][1])
        if distance > max_distance:
            break
        else:
            final_index = i
    return points[0:final_index]

def normalized_points(points, height=500, width=500):
    """
    Takes a list of points (tuples of x, y coordinates) and an output image size
    and returns a transformed list of points that fit in the output image.
    """
    top_left = (min([x for (x, y) in points]), min([y for (x, y) in points]))
    bottom_right = (max([x for (x, y) in points]), max([y for (x, y) in points]))
    x_range = abs(top_left[0] - bottom_right[0])
    y_range = abs(top_left[1] - bottom_right[1])
    return [((x - top_left[0]) * width / x_range, 
        (y - top_left[1]) * height / y_range) 
        for (x, y) in points]

def window(image, location, degree_heading, height=500, width=500):
    """
    Takes an image, a location, an angle in degrees, and optionally a height and width
    in order to return the angled rectangular region of the image of the specified size,
    with the location specifying the bottom middle point of the image.
    ----------------------------------------------------------------------------
    image: 2D array (NumPy/OpenCV) representing the path (should be normalized already)
    location: tuple of (x, y) representing the current location to center the bottom of
              the window at in pixels
    degree_heading: angle in degrees to rotate rectangular region by (counterclockwise)
    height, width: dimensions of the output image in pixels
    ----------------------------------------------------------------------------
    """
    angle = math.radians(degree_heading)
    parallel = (math.cos(angle), math.sin(angle))
    perpendicular = (-math.sin(angle), math.cos(angle))
    horizontal = location[0] - parallel[0] * (width/2) - perpendicular[0] * (height/1)
    vertical = location[1] - parallel[1] * (width/2) - perpendicular[1] * (height/1)
    rotation_matrix = np.array([[parallel[0], perpendicular[0], horizontal], [parallel[1], perpendicular[1], vertical]])
    return cv2.warpAffine(image, rotation_matrix, (width, height), flags=cv2.WARP_INVERSE_MAP)

def dimensions(points):
    """
    Takes a set of latitude and longitude points and returns the width and height in kilometers.
    """
    x_vals = [x for (x, y) in points]
    y_vals = [y for (x, y) in points]
    top_left = (min(x_vals), min(y_vals))
    bottom_right = (max(x_vals), max(y_vals))
    x_range = maps.haversine(top_left[0], top_left[1], bottom_right[0], top_left[1])
    y_range = maps.haversine(top_left[0], top_left[1], top_left[0], bottom_right[1])
    return y_range, x_range

def main():
    # this is just for testing
    points = [(y, -x) for (x, y) in maps.get_points("Mexico City, MX", "New York, NY")] # get google maps lat/longs
    x_vals = [x for (x, y) in points]
    y_vals = [y for (x, y) in points]
    top_left = (min(x_vals), max(y_vals))
    bottom_right = (max(x_vals), min(y_vals))
    y_scale, x_scale = dimensions(points) # this gives the size of our full image
    normalized = normalized_points(points, int(x_scale), int(y_scale)) # let's normalize the points to full image size
    angles = [random.randint(-3, 3) for n in normalized] # generating some random angles because i'm lazy
    full = interpolate([(int(round(x)), int(round(y))) for (x, y) in normalized], 3, 3, int(x_scale), int(y_scale)) # pretend this doesn't look gross
    i = 0
    for n in normalized: # we're iterating through all the points in the line to simulate movement
        current = (int(round(n[0])), int(round(n[1])))
        # swap the two lines below to see what happens with/without angling the image
        rot = window(full, (current[0], current[1]), angles[i], 500, 500)
        #rot = window(full, (current[0], current[1]), 0, 500, 500)
        i += 1
        cv2.imshow("line", rot)
        cv2.waitKey(30) # cinematic

if __name__ == '__main__': main()
