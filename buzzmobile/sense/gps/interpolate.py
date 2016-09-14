import numpy as np
import cv2

"""
takes a list of points, connects them with a line, blurs the line
-------------------------------------------------------------
length, width: length and width of the output image in pixels
points: list of tuples, i.e. (x, y) points that will be plotted on the image
sigma_x, sigma_y: gaussian kernel paramters (higher = more blurry)
"""
def interpolate(length, width, points, sigma_x, sigma_y):
    output = np.zeros((length, width), np.uint8)
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
    points = [(0, 0), (500, 500)]
    interpolate(500,500,1,points, 3, 3, 3)

if __name__ == '__main__': main()
