#!/usr/bin/env python

import cv2
import requests
import rospy

import datetime as dt
import googlemapskey as gmpskey
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import String



g = {} # globals
pub = rospy.Publisher('route_map', Image, queue_size=1)
bridge = CvBridge()


def get_map_url(polyline, coords):
    return ('https://maps.googleapis.com/maps/api/staticmap?size=400x400' +
            '&key=' + gmpskey.googlemapskey +
            '&path=weight:3%7Ccolor:blue%7Cenc:' + polyline +
            '&markers=icon:https://i.imgur.com/DD11yLv.png|color:blue%7Clabel:B%7C'+
            '{0},%20{1}'.format(*coords))

def get_map(polyline, coords):
    url = get_map_url(polyline, coords)
    resp = requests.get(url)
    image = cv2.imdecode(np.asarray(bytearray(resp.content)),
                         cv2.IMREAD_ANYCOLOR)
    return image

def publish_map():
    if g['polyline'] is not None and g['fix'] is not None:
        g['last_published'] = dt.datetime.now()
        polyline = g['polyline'].data
        coords = g['fix'].latitude, g['fix'].longitude
        route_map = get_map(polyline, coords)
        route_msg = bridge.cv2_to_imgmsg(route_map, encoding='bgr8')
        pub.publish(route_msg)

def update_polyline(new_poly):
    g['polyline'] = new_poly
    publish_map()

def update_location(new_fix):
    g['fix'] = new_fix
    if published_long_ago(): publish_map()

def published_long_ago():
    seconds_elapsed = (dt.datetime.now() - g['last_published']).total_seconds()
    return True if seconds_elapsed > 8 else False

def route_mapper_node():
    g['last_published'] = dt.datetime(2012, 10, 23)
    g['polyline'] = None
    g['fix'] = None

    rospy.init_node('route_mapper')
    rospy.Subscriber('polyline', String, update_polyline)
    rospy.Subscriber('/fix', NavSatFix, update_location)
    rospy.spin()

if __name__ == '__main__': route_mapper_node()
