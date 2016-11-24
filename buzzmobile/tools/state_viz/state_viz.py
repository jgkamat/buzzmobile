#!/usr/bin/env python
"""state_viz: node that publishes current car state image for mission_control.

Subscribes:
    car_pose CarPose command which also stores car state (mode)
Publishes:
    state_viz Image representing current car state
"""

import cv2
import os
import rospy

from buzzmobile.msg import CarPose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from time import sleep


g = {} # globals
g['start'] = g['auto'] = g['manual'] = g['neutral'] = None
pub = rospy.Publisher('car_state_image', Image, queue_size=1)

def publish_car_state_image(car_pose):
    """Publishes the correct state image given a car_pose.mode."""
    if car_pose.mode is not None and car_pose.mode in g.keys():
        pub.publish(g[car_pose.mode])

def create_images():
    """Create the state images in memory for fast publishing."""
    base_path = os.path.dirname(os.path.realpath(__file__))
    bridge = CvBridge()

    for image_name in ('start', 'auto', 'manual', 'neutral'):
        img = cv2.imread(os.path.join(base_path, image_name + '.png'))
        g[image_name] = bridge.cv2_to_imgmsg(img, "bgr8")

def state_viz_node():
    """Initializes state_viz node."""
    rospy.init_node('state_viz', anonymous=True)
    create_images()
    sleep(1)
    pub.publish(g['start']) # publish initial state as start
    rospy.Subscriber('car_pose', CarPose, publish_car_state_image)
    rospy.spin()

if __name__ == '__main__': state_viz_node()
