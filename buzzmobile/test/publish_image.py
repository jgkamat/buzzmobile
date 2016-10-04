#!/usr/bin/env python

import sys
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def publish_image(image_filename):
    imgFile = cv2.imread(image_filename)
    imgMsg = bridge.cv2_to_imgmsg(imgFile, "bgr8")

    pub = rospy.Publisher('test_image', Image, queue_size=0)

    rospy.init_node('image_publisher', anonymous=True)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        #rospy.loginfo(imgMsg)
        pub.publish(imgMsg)
        rate.sleep()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Please provide a image filename"
    else:
        image_filename = sys.argv[1]
        publish_image(image_filename)
