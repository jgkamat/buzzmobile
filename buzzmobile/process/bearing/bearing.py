#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from calculate_directions import get_distance, get_forward_angle

class MedianFilter:
    def __init__(self, size):
        self.size = size
        self.array = []

    def add(self, item):
        if len(self.array) > self.size:
            self.array.pop(0)
        self.array.append(item)

    def median(self):
        sortedArray = sorted(self.array)
        index = (len(self.array) - 1) // 2

        if len(self.array) % 2:
            return sortedArray[index]
        else:
            return (sortedArray[index] + sortedArray[index + 1]) / 2.0


g = {} # globals
g['last_fix'] = None
g['med_filter'] = MedianFilter(rospy.get_param('median_filter_size'))
bearing_pub = rospy.Publisher('bearing', Float64, queue_size=1)

MIN_FIX_DISTANCE = rospy.get_param('min_fix_distance')
EARTH_RADIUS = 6.3710088e6


def bearing(fix):
    """
    Adds the latest computed bearing to the meadian filter, publishes
    the current median bearing, and sets last_fix to the given fix
    if the given fix is >= MIN_FIX_DISTANCE from last_fix or there is
    currently no last_fix
    """

    if fix is not None:
        if g['last_fix'] is not None:
            distance = get_distance(g['last_fix'], fix)
            current_bearing = get_forward_angle(g['last_fix'], fix)
            g['med_filter'].add(current_bearing)
            bearing_pub.publish(g['med_filter'].median())
            if distance >= MIN_FIX_DISTANCE:
                g['last_fix'] = fix
        else:
            g['last_fix'] = fix


def bearing_node():
    rospy.init_node('bearing', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, bearing)
    rospy.spin()

if __name__=='__main__': bearing_node()
