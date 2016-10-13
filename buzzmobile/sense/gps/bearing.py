import rospy
import math
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

#Global Variables
last_fix = None
last_bearing = 0
bearing_pub = rospy.Publisher('bearing', Float64, queue_size=0)

def bearing(fix):
    if last_fix is not None:
        lat1 = math.radians(last_fix.latitude)
        lon1 = math.radians(last_fix.longitude)
        lat2 = math.radians(fix.latitude)
        lon2 = math.radians(fix.longitude)
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.cos(lat1)*math.sin(lat2)*math.cos(lon2 - lon1)
        shift = math.atan2(y, x)
        return shift
    else:
        bearing_pub.publish(last_bearing)

def compute_bearing():
    rospy.init_node('compute_bearing', anonymous=True)
    rospy.Subscriber('fix', NavSatFix, bearing)
    rospy.spin()
