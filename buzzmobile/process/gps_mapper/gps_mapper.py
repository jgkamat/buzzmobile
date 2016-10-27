#!/usr/bin/env python

import rospy
import math
import polyline as pl
import interpolate
from collections import namedtuple
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String

#Global Variables
frames = {}
# 'bearing' is counter-clockwise angle from north in radians.
# 'points' will contain the polyline list of lat-lon points
# (normalized to some size based on pixels_per_m).
# 'location' is our lat-lon position (unmodified) from our GPS.
frames['bearing'] = frames['points'] = frames['location'] = None
# These ranges are km dimensions of the path. We initialize them to 0.
frames['y_range'] = frames['x_range'] = 0
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
    """Given a polyline as a ROS message, we normalize and store the points."""
    if polyline is not None:
        points = pl.decode(polyline.data)
        # We calculate the y and x ranges here (in kilometers).
        # This is because we must later flip the polyline points
        # (this is the result of the structure of a polyline), which
        # makes the y and x range calculations invalid later on.
        (y_range, x_range,
         top_left, bottom_right) = interpolate.dimensions(points)
        # These y and x ranges are now stored.
        frames['y_range'] = y_range
        frames['x_range'] = x_range
        # Now, we flip the points because we want the top of the image
        # to represent north.
        frames['points'] = [(y, -x) for (x, y) in points]
        # Because we must flip the points, we can't calculate the y and x
        # ranges accurately in one step. This is why we do not reset the 
        # y and x ranges here. However, we do still need the top left and
        # bottom right coordinates.
        _, _, top_left, bottom_right = interpolate.dimensions(frames['points'])
        # Based on the accurate y and x ranges, we calculate a height and width
        # that will scale our final image to our specified pixels_per_m
        # (pixels per meter) parameter.
        height = int(round(frames['y_range'] * y_scale))
        width = int(round(frames['x_range'] * x_scale))
        # We store the normalized points.
        frames['points'] = interpolate.normalized_points(frames['points'],
                                                         width,
                                                         height)

def update_image():
    """
    Called every time the location of the car or the bearing of the car changes.
    """
    if frames['points'] is not None:
        _, _, top_left, bottom_right = interpolate.dimensions(frames['points'])
        height = int(round(frames['y_range'] * y_scale))
        width = int(round(frames['x_range'] * x_scale))
        point = (frames['location'][0], -frames['location'][1])
        if frames['y_range'] is not 0:
            point = interpolate.normalize_single_point(frames['y_range'],
                                                       frames['x_range'],
                                                       height, width,
                                                       top_left, bottom_right,
                                                       point)
        result = interpolate.xwindow(frames['points'], point, frames['bearing'],
                                     line_width,
                                     sigma_x,
                                     sigma_y,
                                     image_height,
                                     image_width)
        result_msg = bridge.cv2_to_imgmsg(result, encoding='mono8')
        gps_model_pub.publish(result_msg)

def set_bearing(angle):
    frames['bearing'] = angle.data
    if frames['location'] is not None:
        update_image()

def set_location(fix_location):
    frames['location'] = (fix_location.latitude, fix_location.longitude)
    if frames['bearing'] is not None:
        update_image()

def gps_mapper_node():
    rospy.init_node('gps_mapper', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('/fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': gps_mapper_node()
