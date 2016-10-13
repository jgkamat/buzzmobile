#!/usr/bin/env python

import rospy
import math
import polyline as pl
import interpolate
import maps
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String

class Frames(object):
    def __init__(self):
        bearing = None
        points = None
        location = None
        full = None

#Global Variables
bridge = CvBridge()
frames = Frames()
gps_model_pub = rospy.Publisher('gps_model', Image, queue_size=0)
x_scale = y_scale = 5000

def set_points(polyline):
    if polyline is not None:
        points = pl.decode(polyline.data)
        frames.points = points
        y_range, x_range = interpolate.dimensions(frames.points)
        y_range *= y_scale
        x_range *= x_scale
        normalized = interpolate.normalized_points(points, int(x_range), int(y_range))
        frames.full = interpolate.interpolate([(int(round(x)), int(round(y))) for (x, y) in normalized], 3, 3, int(x_range), int(y_range))

def update_image():
    if hasattr(frames, 'full'):
        y_range, x_range = interpolate.dimensions(frames.points)
        height = y_scale * y_range
        width = x_scale * x_range
        lat = x_scale * frames.location[0]
        lon = y_scale * frames.location[1]
        point = (lat, lon)
        point = interpolate.normalize_single_point(y_range, x_range, height, width, point)
        result = interpolate.window(frames.full, point, frames.bearing)
        result_msg = bridge.cv2_to_imgmsg(result)
        gps_model_pub.publish(result_msg)

def set_bearing(angle):
    frames.bearing = angle.data
    if hasattr(frames, 'location'):
        update_image()

def set_location(fix_location):
    frames.location = (fix_location.latitude, fix_location.longitude)
    if hasattr(frames, 'bearing'):
        update_image()

def gps_model_node():
    rospy.init_node('gps_mapper', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': gps_model_node()
