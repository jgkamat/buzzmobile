#!/usr/bin/env python
import googlemaps
import math
import numpy as np
import polyline as pl
import rospy

import googlemapskey as gmpskey

from datetime import datetime
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


# GLOBALS
gmaps = googlemaps.Client(key=gmpskey.googlemapskey)
pub = rospy.Publisher('polyline', String, queue_size=0)
target = {
        'destination': None,
        'published': False,
        'fix': None}


def update_destination(new_destination):
    target['destination'] = new_destination.data
    target['published'] = False
    if target['fix'] is not None:
        publish_polyline()

def update_fix(new_fix):
    target['fix'] = new_fix.latitude, new_fix.longitude
    if (target['destination'] is not None and not target['published']):
        publish_polyline()

def publish_polyline():
    pub.publish(get_polyline(target['fix'], target['destination']))
    target['published'] = True

def get_polyline(start, destination):
    direction_result = get_directions(start, destination)
    return str(direction_result[0]['overview_polyline']['points'])

def get_directions(start, destination):
    """
    Returns a list of current driving directions from start to destination.
    start and destination may be strings or gps coordinates, e.g.,

    >>> get_directions("Atlanta, GA", "Seattle, WA")
    [direction object 1, direction object 2, ...]

    >>> get_directions((20.123, 41.382), (42.000, 3.142))
    [direction object 1, direction object 2, ...]
    """
    now = datetime.now()
    direction_result = gmaps.directions(origin = start,
                                        destination = destination,
                                        alternatives = True,
                                        mode="driving",
                                        units="metric",
                                        departure_time=now)
    return direction_result

def maps_querier_node():
    rospy.init_node('maps_querier')
    rospy.Subscriber('/fix', NavSatFix, update_fix)
    rospy.Subscriber('destination', String, update_destination)
    rospy.spin()

if __name__ == '__main__': maps_querier_node()
