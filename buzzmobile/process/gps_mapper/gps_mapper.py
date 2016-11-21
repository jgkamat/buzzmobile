#!/usr/bin/env python

import interpolate
import math
import polyline as pl
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import String

#Global Variables
g = {} # globals
# 'bearing' is counter-clockwise angle from north in radians.
# 'points' will contain the polyline list of lat-lon points
# (normalized to some size based on pixels_per_m).
# 'fixes' is a list of our last 'median_filter_size' number of lat-lon
# coordinates which are passed through a median filter
# to reduce the effect of noise in our location data.
g['bearing'] = g['points'] = g['location'] = None
g['fixes'] = []
# These ranges are km dimensions of the path. Initialize them to 0.
# Also initialize the scaled width and height of the points to 0.
(g['y_range'] = g['x_range'] = g['height']
 = g['width'] = g['ll_height'] = g['ll_width'] = 0)
# Initialize the top left and bottom right image coordinates to (0, 0).
g['top_left'] = g['bottom_right'] = (0, 0)
# We also have these toggles to see if bearing and location have been updated.
# These are used to in order to sync the updating of the model with updates of
# both bearing and location, so that these hopefully match.
g['location_toggle'] = g['bearing_toggle'] = False
gps_model_pub = rospy.Publisher('gps_model', Image, queue_size=1)
bridge = CvBridge()
x_scale = y_scale = 1000 * rospy.get_param('pixels_per_m')
line_width = int(round(rospy.get_param('pixels_per_m')
                       * rospy.get_param('road_width')))
median_filter_size = rospy.get_param('median_filter_size')
sigma_x = rospy.get_param('sigma_x')
sigma_y = rospy.get_param('sigma_y')
image_width = rospy.get_param('image_width')
image_height = rospy.get_param('image_height')

def set_points(polyline):
    """Given a polyline as a ROS message, normalize and store the points."""
    if polyline is not None:
        points = pl.decode(polyline.data)
        # Calculate the y and x ranges (in kilometers).
        # This is because we must later flip the polyline points
        # (this is because a polyline point is a tuple of (lat, lon) while we
        # want (x, y) image coordinates), which
        # makes the y and x range calculations invalid later on.
        y_range, x_range = interpolate.dimensions(points)
        # Store y and x ranges.
        g['y_range'] = y_range
        g['x_range'] = x_range
        # Flip the points because we want the top of the image
        # to represent north.
        g['points'] = [(y, -x) for (x, y) in points]
        # Calculate top left and bottom right coordinates
        # after flipping the polyline points.
        top_left, bottom_right = interpolate.corners(g['points'])
        g['top_left'] = top_left
        g['bottom_right'] = bottom_right
        g['ll_height'] = abs(top_left[1] - bottom_right[1])
        g['ll_width'] = abs(top_left[0] - bottom_right[0])
        # Based on the accurate y and x ranges, calculate a height and width
        # that will scale our final image to our specified pixels_per_m
        # (pixels per meter) parameter.
        # Note that this is distinct from `image_height` and `image_width`,
        # which are the height and width of the current window that will be
        # passed to the frame merger!
        g['height'] = int(round(g['y_range'] * y_scale))
        g['width'] = int(round(g['x_range'] * x_scale))
        # Store the normalized points.
        g['points'] = interpolate.normalized_points(g['points'],
                      g['top_left'],
                      g['ll_height'],
                      g['ll_width']
                      g['height'],
                      g['width'])

def update_image():
    """
    Called every time the location of the car or the bearing of the car changes.
    Calculates the rotated point space and then interpolates the points in the
    specified window before publishing this updated window (image) to ROS.
    """
    if g['points'] is not None and g['location_toggle'] and g['bearing_toggle']:
        g['location_toggle'] = g['bearing_toggle'] = False
        # Calculate the top left and bottom right points
        # of the full list of points.
        # Normalize the current location to the size of the
        # normalized polyline points.
        point = median_filter(g['location'])
        point = (point[1], -point[0])
        if g['y_range'] is not 0 and point is not None:
            point = interpolate.normalize_single_point(point, g['top_left'],
                    g['ll_height'],
                    g['ll_width'],
                    g['height'],
                    g['width'])
            # Call method to calculate rotated points and interpolate a path
            # between the points that are in the current window
            # (based on the current location and bearing).
            result = interpolate.window(g['points'], point, g['bearing'],
                     line_width,
                     sigma_x,
                     sigma_y,
                     image_height,
                     image_width)
            # Send the final image window as an image message through ROS.
            result_msg = bridge.cv2_to_imgmsg(result, encoding='mono8')
            gps_model_pub.publish(result_msg)

def set_bearing(angle):
    """Given a radian bearing, update the current bearing and update image."""
    if angle is not None:
        g['bearing'] = angle.data
        if g['location'] is not None:
            g['bearing_toggle'] = True
            update_image()

def set_location(fix_location):
    """Given a lat-lon, update current location and update image."""
    if fix_location is not None:
        g['location'] = (fix_location.latitude, fix_location.longitude)
        if g['bearing'] is not None:
            g['location_toggle'] = True
            update_image()

def median_filter(fix):
    if len(g['fixes']) > median_filter_size:
        g['fixes'].pop(0)
    g['fixes'].append(fix)
    sorted_points = sorted(g['fixes'])
    index = len(g['fixes']) // 2

    if len(g['fixes']) % 2 and len(g['fixes']) > 0:
        return sorted_points[index]
    elif len(g['fixes']) > 1:
        return ((sorted_points[index][0] + sorted_points[index + 1][0]) * 0.5,
                (sorted_points[index][1] + sorted_points[index + 1][1]) * 0.5)
    elif len(g['fixes'] == 1):
        return sorted_points[0]
    else:
        return None

def gps_mapper_node():
    """
    Initializes the ROS node and starts listening for
    polylines, bearings, and locations.
    """
    rospy.init_node('gps_mapper', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('/fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': gps_mapper_node()
