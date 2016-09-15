import numpy as np
import cv2
from maps import get_points


def interpolate(height=500, width=500, points, sigma_x, sigma_y):
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
    cv2.imshow("line", output)
    cv2.waitKey(0)

def main():
    # this is just for testing
    points = [(y, -x) for (x, y) in get_points("Mexico City, MX", "New York, NY")]
    interpolate(500, 500, [(round(x), round(y)) for (x, y) in normalized_points(points)], 3, 3)

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

if __name__ == '__main__': main()