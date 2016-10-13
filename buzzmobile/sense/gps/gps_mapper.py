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

def set_points(polyline_string):
    if polyline_string is not None:
        points = pl.decode(polyline_string)
        frames.points = points
        y_range, x_range = interpolate.dimensions(frames.points)
        y_range *= y_scale
        x_range *= x_scale
        normalized = interpolate.normalized_points(points, int(x_range), int(y_range))
        frames.full = interpolate.interpolate([(int(round(x)), int(round(y))) for (x, y) in normalized], 3, 3, int(x_range), int(y_range))

def update_image():
    if frames.full is not None:
        result = window(frames.full, frames.location, frames.bearing)
        result_msg = bridge.cv2_to_imgmsg(result)
        gps_model_pub.publish(result_msg)

def set_bearing(angle):
    frames.bearing = angle
    if frames.location is not None:
        update_image()

def set_location(fix_location):
    frames.location = fix_location
    if frames.bearing is not None:
        update_image(result)

def construct_gps_model():
    rospy.init_node('construct_gps_model', anonymous=True)
    rospy.Subscriber('polyline', String, set_points)
    rospy.Subscriber('bearing', Float64, set_bearing)
    rospy.Subscriber('fix', NavSatFix, set_location)
    rospy.spin()

if __name__=='__main__': construct_gps_model()
