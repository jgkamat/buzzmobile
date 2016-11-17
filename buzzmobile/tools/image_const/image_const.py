#!/usr/bin/env python
"""image_const: node that continuously publishes a given image from file.

Publishes:
    image_filename Image with topic name same as file name
"""

from __future__ import print_function

import sys
import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


bridge = CvBridge()


def continuously_publish_image(image_filename):
    """Publishes the image given at a constant rate."""
    cv2_img = cv2.imread(image_filename)
    gray_im = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    img_msg = bridge.cv2_to_imgmsg(gray_im, encoding="mono8")

    topic_name = sys.argv[1].split('/')[-1][:-4]
    pub = rospy.Publisher(topic_name, Image, queue_size=1)

    rospy.init_node('image_const', anonymous=True)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        pub.publish(img_msg)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please provide a image filename")
    else:
        continuously_publish_image(sys.argv[1])
