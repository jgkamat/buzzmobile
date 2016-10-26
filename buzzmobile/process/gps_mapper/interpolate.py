import numpy as np
import math
import cv2

def interpolate(points, sigma_x, sigma_y, height, width):
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

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculates the distance between two lat-lon points in kilometers.
    """
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    a = math.sin(delta_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371
    d = R * c
    return d

def normalized_points(points, height, width):
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
    
def normalize_single_point(y_range, x_range, height, width, top_left, bottom_right, point):
    return ((point[0] - top_left[0]) * width / x_range, (point[1] - top_left[1]) * height / y_range)


def window(image, location, angle, height, width):
    """
    DO NOT USE THIS. USE THE FOLLOWING FUNCTION INSTEAD, WHICH TAKES ONLY POINTS INSTEAD OF
    A FULL IMAGE. THE FOLLOWING FUNCTION IS LESS MEMORY INTENSIVE.
    Takes an image, a location, an angle in radians, and optionally a height and width
    in order to return the angled rectangular region of the image of the specified size,
    with the location specifying the bottom middle point of the image.
    ----------------------------------------------------------------------------
    image: 2D array (NumPy/OpenCV) representing the path (should be normalized already)
    location: tuple of (x, y) representing the current location to center the bottom of
              the window at in pixels
    angle: angle in radians to rotate rectangular region by (counterclockwise)
    height, width: dimensions of the output image in pixels
    ----------------------------------------------------------------------------
    """
    parallel = (math.cos(angle), math.sin(angle))
    perpendicular = (-math.sin(angle), math.cos(angle))
    horizontal = location[0] - parallel[0] * (width/2) - perpendicular[0] * (height/1)
    vertical = location[1] - parallel[1] * (width/2) - perpendicular[1] * (height/1)
    rotation_matrix = np.array([[parallel[0], perpendicular[0], horizontal], [parallel[1], perpendicular[1], vertical]])
    return cv2.warpAffine(image, rotation_matrix, (width, height), flags=cv2.WARP_INVERSE_MAP)

def xwindow(points, location, angle, height, width):
    """
    Takes a list of points, a location, an angle in radians, and optionally a height and width
    in order to return the angled rectangular region of the image of the specified size,
    with the location specifying the bottom middle point of the image. The image itself is
    the blurred interpolation of all the points (we only construct the relevant portion of the
    rotated image).
    ----------------------------------------------------------------------------
    points: list of (x, y) tuples representing points
    location: tuple of (x, y) representing the current location to center the bottom of
              the window at in pixels
    angle: angle in radians to rotate rectangular region by (counterclockwise)
    height, width: dimensions of the output image in pixels
    ----------------------------------------------------------------------------
    """
    out = []
    parallel = (math.cos(angle), -math.sin(angle))
    perpendicular = (math.sin(angle), math.cos(angle))
    for (x, y) in points:
        x_ = x * parallel[0] + y * parallel[1] + location[0] - location[0] * parallel[0] - location[1] * parallel[1]
        y_ = x * perpendicular[0] + y * perpendicular[1] + location[1] - location[0] * perpendicular[0] - location[1] * perpendicular[1]
        # when adding the points back, translate them to the proper location for the final image
        out.append((x_ + (width/2 - location[0]), y_ + (height - location[1])))
    return interpolate([(int(round(x)), int(round(y))) for (x, y) in out], 3, 3, width, height)

def dimensions(points):
    """
    Takes a set of latitude and longitude points and returns the width and height in kilometers.
    """
    x_vals = [x for (x, y) in points]
    y_vals = [y for (x, y) in points]
    top_left = (min(x_vals), max(y_vals))
    bottom_right = (max(x_vals), min(y_vals))
    x_range = haversine(top_left[0], top_left[1], bottom_right[0], top_left[1])
    y_range = haversine(top_left[0], top_left[1], top_left[0], bottom_right[1])
    return y_range, x_range, top_left, bottom_right