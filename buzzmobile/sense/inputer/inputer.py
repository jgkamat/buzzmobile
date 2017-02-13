#!/usr/bin/env python
"""inputer: REPL for inputting destinations for the car.

Publishes:
    destination String the destination inputted via the command line
"""

import rospy

from std_msgs.msg import String
from buzzmobile.msg import CarState


def inputer_node():
    """Initializes inputer node."""
    pub = rospy.Publisher('destination', String, queue_size=1)
    rospy.init_node('inputer', anonymous=True)
    # Publish a nonsense initial value to make sure subscribers don't explode
    pub.publish("")
    while not rospy.is_shutdown():
        new_dst = raw_input("Dest >> ")
        pub.publish(new_dst)

if __name__ == '__main__': inputer_node()
