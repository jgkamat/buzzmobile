#!/usr/bin/env python
import cv2
import os
import rospy
import sys

from buzzmobile.msg import CarState
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


g = {} # globals
g['start'] = g['auto'] = g['manual'] = g['neutral'] = None
pub = rospy.Publisher('car_state_image', Image, queue_size=1)

def publish_car_state_image(car_state):
    if car_state.state == CarState.START: pub.publish(g['start'])
    elif car_state.state == CarState.AUTO: pub.publish(g['auto'])
    elif car_state.state == CarState.MANUAL: pub.publish(g['manual'])
    # TODO(irapha): add neutral car state
    # elif car_state == CarState.NEUTRAL: pub.publish(g['neutral'])

def create_images():
    BASE_PATH = os.path.dirname(os.path.realpath(__file__))
    bridge = CvBridge()

    for image_name in ('start', 'auto', 'manual', 'neutral'):
        img = cv2.imread(os.path.join(BASE_PATH, image_name + '.png'))
        g[image_name] = bridge.cv2_to_imgmsg(img, "bgr8")

def state_viz_node():
    create_images()
    rospy.init_node('state_viz', anonymous=True)
    rospy.Subscriber('car_state', CarState, publish_car_state_image)
    pub.publish(g['start']) # publish initial state as start
    rospy.spin()

if __name__ == '__main__': state_viz_node()
