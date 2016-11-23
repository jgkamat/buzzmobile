#!/usr/bin/env python
"""route_mapper node: publishes map image with current location and route.

Subscribes:
    /fix NavSatFix the current location
    polyline String the route to follow
Publishes:
    route_map Image map with current location and route
"""

import datetime as dt
import requests
import rospy

import cv2
import googlemapskey as gmpskey
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import String


g = {} # globals
g['fix'] = g['polyline'] = None
g['last_published'] = dt.datetime(2012, 10, 23)
pub = rospy.Publisher('route_map', Image, queue_size=1)
bridge = CvBridge()

def get_map_url(polyline, coords):
    """Returns URL for google maps image with the route and coords."""
    return ('https://maps.googleapis.com/maps/api/staticmap?size=400x400' +
            '&key=' + gmpskey.googlemapskey +
            '&path=weight:3%7Ccolor:blue%7Cenc:' + polyline +
            '&markers=icon:https://i.imgur.com/DD11yLv.png' +
            '|color:blue%7Clabel:B%7C' +
            '{0},%20{1}'.format(*coords))

def get_map(polyline, coords):
    """Queries google maps and returns map image of route and coords."""
    url = get_map_url(polyline, coords)
    resp = requests.get(url)
    image = cv2.imdecode(np.asarray(bytearray(resp.content)),
                         cv2.IMREAD_ANYCOLOR)
    return image

def publish_map():
    """If possible, get and publish map topic with updated polyline/coords."""
    if g['polyline'] is not None and g['fix'] is not None:
        g['last_published'] = dt.datetime.now()
        polyline = g['polyline'].data
        coords = g['fix'].latitude, g['fix'].longitude
        route_map = get_map(polyline, coords)
        route_msg = bridge.cv2_to_imgmsg(route_map, encoding='bgr8')
        pub.publish(route_msg)

def update_polyline(new_poly):
    """Update route polyline, and publishes map."""
    g['polyline'] = new_poly
    publish_map()

def update_location(new_fix):
    """Update current location, and publishes map if it has been a while"""
    g['fix'] = new_fix
    if published_long_ago(): publish_map()

def published_long_ago():
    """Returns whether map was last published over 8 seconds ago."""
    secs_elapsed = (dt.datetime.now() - g['last_published']).total_seconds()
    return True if secs_elapsed > 8 else False

def route_mapper_node():
    """Initializes route_mapper node."""
    rospy.init_node('route_mapper')
    rospy.Subscriber('polyline', String, update_polyline)
    rospy.Subscriber('/fix', NavSatFix, update_location)
    rospy.spin()

if __name__ == '__main__': route_mapper_node()
