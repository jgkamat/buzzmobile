#!/usr/bin/env python
import rospy

from std_msgs.msg import String


def input_node():
    pub = rospy.Publisher('destination', String, queue_size=1)
    rospy.init_node('inputer', anonymous=True)
    # Publish a nonsense initial value to make sure subscribers don't explode
    pub.publish("")
    while not rospy.is_shutdown():
        new_dst = raw_input("Dest >> ")
        pub.publish(new_dst)

if __name__ == '__main__': input_node()
