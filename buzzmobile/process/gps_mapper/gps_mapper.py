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
bridge = CvBridge()
frames = {}
frames['bearing'] = frames['points'] = frames['location'] = None
gps_model_pub = rospy.Publisher('gps_model', Image, queue_size=1)
x_scale = y_scale = 1000 * rospy.get_param('pixels_per_m')
y_range = x_range = 0

def set_points(polyline):
    if polyline is not None:
        points = pl.decode(polyline.data)
        y_range, x_range, top_left, bottom_right = interpolate.dimensions(frames['points'])
        frames['points'] = [(y, -x) for (x, y) in points]
        _, _, top_left, bottom_right = interpolate.dimensions(frames['points'])
        height = y_range * y_scale
        width = x_range * x_scale
        frames['points'] = interpolate.normalized_points(frames['points'], int(round(width)), int(round(height)))

def update_image():
    if frames['points'] is not None:
        _, _, top_left, bottom_right = interpolate.dimensions(frames['points'])
        height = int(round(y_range * y_scale))
        width = int(round(x_range * x_scale))
        point = (frames['location'][0], -frames['location'][1])
        if y_range is not 0:
            point = interpolate.normalize_single_point(y_range, x_range, height, width, top_left, bottom_right, point)
        result = interpolate.xwindow(frames['points'], point, frames['bearing'], rospy.get_param('image_height'), rospy.get_param('image_width'))
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
