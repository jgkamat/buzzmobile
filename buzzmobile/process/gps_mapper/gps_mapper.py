#!/usr/bin/env python

import rospy
import polyline as pl
import interpolate
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String

#Global Variables
g = {} # globals
# 'bearing' is counter-clockwise angle from north in radians.
# 'points' will contain the polyline list of lat-lon points
# (normalized to some size based on pixels_per_m).
# 'location' is our lat-lon position (unmodified) from our GPS.
g['bearing'] = g['points'] = g['location'] = None
# These ranges are km dimensions of the path. Initialize them to 0.
# Also initialize the scaled width and height of the points to 0.
g['y_range'] = g['x_range'] = g['height'] = g['width'] = 0
gps_model_pub = rospy.Publisher('gps_model', Image, queue_size=1)
bridge = CvBridge()
x_scale = y_scale = 1000 * rospy.get_param('pixels_per_m')
line_width = int(round(rospy.get_param('pixels_per_m')
                       * rospy.get_param('road_width')))
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
        (y_range, x_range,
         top_left, bottom_right) = interpolate.dimensions(points)
        # Store y and x ranges.
        g['y_range'] = y_range
        g['x_range'] = x_range
        # Flip the points because we want the top of the image
        # to represent north.
        g['points'] = [(y, -x) for (x, y) in points]
        # Recalculate top left and bottom right coordinates
        # after flipping the polyline points. Do not save the new
        # y and x ranges, because those will not be correct.
        _, _, top_left, bottom_right = interpolate.dimensions(g['points'])
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
                                                    g['width'],
                                                    g['height'])

def update_image():
    """
    Called every time the location of the car or the bearing of the car changes.
    Calculates the rotated point space and then interpolates the points in the
    specified window before publishing this updated window (image) to ROS.
    """
    if g['points'] is not None:
        # Calculate the top left and bottom right points
        # of the full list of points.
        _, _, top_left, bottom_right = interpolate.dimensions(g['points'])
        # Normalize the current location to the size of the
        # normalized polyline points.
        point = (g['location'][1], -g['location'][0])
        if g['y_range'] is not 0:
            point = interpolate.normalize_single_point(g['y_range'],
                    g['x_range'],
                    g['height'],
                    g['width'],
                    top_left,
                    bottom_right,
                    point)
        # Call method to calculate rotated points and interpolate a path
        # between the points that are in the current window
        # (based on the current location and bearing).
        result = interpolate.xwindow(g['points'], point, g['bearing'],
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
            update_image()

def set_location(fix_location):
    """Given a lat-lon, update current location and update image."""
    if fix_location is not None:
        g['location'] = (fix_location.latitude, fix_location.longitude)
        if g['bearing'] is not None:
            update_image()

def gps_mapper_node():
    # Initializes the ROS node and starts listening for
    # polylines, bearings, and locations.
    rospy.init_node('gps_mapper', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('/fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': gps_mapper_node()
