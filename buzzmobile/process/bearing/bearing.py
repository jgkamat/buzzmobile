#!/usr/bin/env python
"""bearing: node that publishes the latest calculated bearing from gps fixes.

Subscribes:
    /fix NavSatFix the current location
Publishes:
    bearing Float64 the median calculated bearing
"""

import math
import rospy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from calculate_directions import get_distance, get_forward_angle
from median_filter import MedianFilter

g = {} # globals
g['last_fix'] = None
g['med_filter'] = MedianFilter(rospy.get_param('median_filter_size'))
pub = rospy.Publisher('bearing', Float64, queue_size=1)

MIN_FIX_DISTANCE = rospy.get_param('min_fix_distance')


def bearing(fix):
    """Updates current fix and publishes newest median bearing."""
    if fix is not None:
        if g['last_fix'] is not None:
            distance = get_distance(g['last_fix'], fix)
            current_bearing = get_forward_angle(g['last_fix'], fix)
            g['med_filter'].add(current_bearing)

            # correct bearing published to be 0 rad at north
            pub.publish(2 * math.pi - g['med_filter'].median())

            # only save last fix if the new one is very different from last
            if distance >= MIN_FIX_DISTANCE:
                g['last_fix'] = fix
        else:
            # we just got a first fix.
            g['last_fix'] = fix


def bearing_node():
    """Initializes bearing node."""
    rospy.init_node('bearing', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, bearing)
    rospy.spin()

if __name__ == '__main__': bearing_node()
