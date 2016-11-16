#!/usr/bin/env python
import cv2
import os
import rospy
import sys

from buzzmobile.msg import CarPose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from time import sleep


g = {} # globals
g['start'] = g['auto'] = g['manual'] = g['neutral'] = None
pub = rospy.Publisher('car_state_image', Image, queue_size=1)

def publish_car_state_image(car_pose):
    if car_pose.mode is not None and car_pose.mode in g.keys():
        pub.publish(g[car_pose.mode])

def create_images():
    BASE_PATH = os.path.dirname(os.path.realpath(__file__))
    bridge = CvBridge()

    for image_name in ('start', 'auto', 'manual', 'neutral'):
        img = cv2.imread(os.path.join(BASE_PATH, image_name + '.png'))
        g[image_name] = bridge.cv2_to_imgmsg(img, "bgr8")

def state_viz_node():
    rospy.init_node('state_viz', anonymous=True)
    create_images()
    sleep(1)
    pub.publish(g['start']) # publish initial state as start
    rospy.Subscriber('car_pose', CarPose, publish_car_state_image)
    rospy.spin()

if __name__ == '__main__': state_viz_node()
