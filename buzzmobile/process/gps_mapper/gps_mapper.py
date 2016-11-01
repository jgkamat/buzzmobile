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


bridge = CvBridge()
g = {} # globals
g['bearing'] = g['points'] = g['location'] = None
# These ranges are km dimensions of the path. We initialize them to 0.
g['y_range'] = g['x_range'] = 0
gps_model_pub = rospy.Publisher('gps_model', Image, queue_size=1)
x_scale = y_scale = 1000 * rospy.get_param('pixels_per_m')

def set_points(polyline):
    if polyline is not None:
        points = pl.decode(polyline.data)
        y_range, x_range, top_left, bottom_right = interpolate.dimensions(g['points'])
        g['y_range'] = y_range
        g['x_range'] = x_range
        g['points'] = [(y, -x) for (x, y) in points]
        _, _, top_left, bottom_right = interpolate.dimensions(g['points'])
        height = g['y_range'] * y_scale
        width = g['x_range'] * x_scale
        g['points'] = interpolate.normalized_points(g['points'], int(round(width)), int(round(height)))

def update_image():
    if g['points'] is not None:
        _, _, top_left, bottom_right = interpolate.dimensions(g['points'])
        height = int(round(g['y_range'] * y_scale))
        width = int(round(g['x_range'] * x_scale))
        point = (g['location'][0], -g['location'][1])
        if g['y_range'] is not 0:
            point = interpolate.normalize_single_point(g['y_range'], g['x_range'], height, width, top_left, bottom_right, point)
        result = interpolate.xwindow(g['points'], point, g['bearing'], rospy.get_param('image_height'), rospy.get_param('image_width'))
        result_msg = bridge.cv2_to_imgmsg(result, encoding='mono8')
        gps_model_pub.publish(result_msg)

def set_bearing(angle):
    g['bearing'] = angle.data
    if g['location'] is not None:
        update_image()

def set_location(fix_location):
    g['location'] = (fix_location.latitude, fix_location.longitude)
    if g['bearing'] is not None:
        update_image()

def gps_mapper_node():
    rospy.init_node('gps_mapper', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('/fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': gps_mapper_node()
