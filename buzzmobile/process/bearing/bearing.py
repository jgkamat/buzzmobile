#!/usr/bin/env python

#TODO: Don't update fix if last_fix is within epsilon of current fix.

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
last_fix = None
med_filter = MedianFilter(rospy.get_param('median_filter_size'))
bearing_pub = rospy.Publisher('bearing', Float64, queue_size=1)

def bearing(fix):
    global last_fix
    global med_filter

    if last_fix is not None:
        lat1 = math.radians(last_fix.latitude)
        lon1 = math.radians(last_fix.longitude)
        lat2 = math.radians(fix.latitude)
        lon2 = math.radians(fix.longitude)
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(lon2 - lon1)
        angle = math.atan2(y, x)
        computedBearing = (angle + 2*math.pi) % 2*math.pi
        med_filter.add(computedBearing)

        bearing_pub.publish(med_filter.median())

    if fix is not None:
        last_fix = fix

def bearing_node():
    rospy.init_node('bearing', anonymous=True)
    rospy.Subscriber('/fix', NavSatFix, bearing)
    rospy.spin()

if __name__=='__main__': compute_bearing_node()
