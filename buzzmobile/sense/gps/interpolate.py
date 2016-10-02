import numpy as np
import cv2
import maps
#import rospy

def interpolate(points, sigma_x, sigma_y, height=500, width=500):
    """
    Takes a list of points, connects them with a line,
    and Gaussian blurs the line.
    ----------------------------------------------------------------------------
    height, width: dimensions of the output image in pixels
    points: list of tuples, i.e. (x, y) points that will be plotted on the image
    sigma_x, sigma_y: gaussian kernel paramters (higher = more blurry)
    ----------------------------------------------------------------------------
    """
    output = np.zeros((height, width), np.uint8)
    x = [x for (x, y) in points]
    y = [y for (x, y) in points]
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(output, pt1, pt2, [255, 255, 255], 4)
    output = cv2.GaussianBlur(output, (sigma_x, sigma_y), 0)
    return output
    pub = rospy.Publisher('gps_image', Image, queue_size = 10)
    rospy.init_node('gps', anonymous=True)
    #cv2.imshow("line", output)
    #cv2.waitKey(0)

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

def centered_normalized_points(points, height=500, width=500):
    """
    Takes a list of points (tuples of x, y coordinates) and an output image size
    and returns a transformed list of points that fit in the output image such that
    the bottom-most point is centered with respect to the output width.
    """
    x_vals = [x for (x, y) in points]
    y_vals = [y for (x, y) in points]
    top_left = (min(x_vals), min(y_vals))
    bottom_right = (max(x_vals), max(y_vals))
    first = points[0]
    x_range = abs(top_left[0] - bottom_right[0])
    y_range = abs(top_left[1] - bottom_right[1])
    y_vals = [(y - top_left[1]) * height / y_range for (x, y) in points]
    x_vals = [((x - first[0]) * (width / (2 * x_range)) + width / 2) for (x, y) in points]
    return [(x_vals[i], y_vals[i]) for i in range(len(points))]

def main():
    # this is just for testing
    # get points from Google Maps and also make a numpy array
    points = [(y, -x) for (x, y) in maps.get_points("Mexico City, MX", "New York, NY")]
    points_array = np.asarray(points)
    start_index = 0 # index of closest point to current location
    current_index = 0 # we're simulating our current location by iterating through points
    window = close_points(points[start_index:], 100000) # initial sliding window from the first point to some max distance
    # we'll have to tweak the exact numbers later
    while len(window) > 1 and window[-1] != points[-1] and current_index < len(points):
        # call interpolate function on rounded point values after normalizing and centering the points
        # pretty sure we don't actually want to normalize in this way but we use it for now just to have centered points
        frame = interpolate([(int(round(x)), int(round(y))) for (x,y) in centered_normalized_points(window)], 3, 3)
        cv2.imshow("line", frame)
        cv2.waitKey(100)
        current_index += 1
        # update the index of the closest point to the new simulated current location
        start_index = nearest_point_index(points[current_index], points_array)
        window = close_points(points[start_index:], 100000)
    cv2.waitKey(0)

if __name__ == '__main__': main()
