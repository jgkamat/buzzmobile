#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

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


#Global Variables
bearings = {}
bearings['last_fix'] = None
bearings['med_filter'] = MedianFilter(rospy.get_param('median_filter_size'))
bearing_pub = rospy.Publisher('bearing', Float64, queue_size=1)

MIN_FIX_DISTANCE = rospy.get_param('min_fix_distance')
EARTH_RADIUS = 6.3710088e6


def bearing(fix):
    """
    Adds the latest computed bearing to the meadian filter, publishes
    the current median bearing, and sets last_fix to the given fix
    if the given fix is >= MIN_FIX_DISTANCE from last_fix
    """

    if fix is not None:
        if bearings['last_fix'] is not None:
            lat1 = math.radians(bearings['last_fix'].latitude)
            lon1 = math.radians(bearings['last_fix'].longitude)
            lat2 = math.radians(fix.latitude)
            lon2 = math.radians(fix.longitude)

            distance = get_distance(lat1, lon1, lat2, lon2)
            computedBearing = get_forward_angle(lat1, lon1, lat2, lon2)
            bearings['med_filter'].add(computedBearing)
            bearing_pub.publish(bearings['med_filter'].median())

            if distance >= MIN_FIX_DISTANCE:
                last_fix = fix
        else:
            last_fix = fix

def get_distance(lat1, lon1, lat2, lon2):
    """
    Calculates great-circle distance between two positions in meters
    """

    a = (math.pow(math.sin(lat2 - lat1), 2)
         + math.cos(lat1) * math.cos(lat2)
         * math.pow(math.sin((lon2 - lon1) / 2), 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c

def get_forward_angle(lat1, lon1, lat2, lon2):
    """
    Calculates forward azimuth between two positions in radians
    """

    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    angle = math.atan2(y, x)
    return (angle + 2 * math.pi) % (2 * math.pi)

def bearing_node():
    rospy.init_node('bearing', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, bearing)
    rospy.spin()

if __name__=='__main__': compute_bearing_node()
