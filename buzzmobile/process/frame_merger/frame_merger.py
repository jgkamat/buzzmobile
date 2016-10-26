#!/usr/bin/env python
import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Frames(object):
    def __init__(self):
        lidar_model = None
        gps_model = None

# Global Variables
bridge = CvBridge()
world_pub = rospy.Publisher('world_model', Image, queue_size=1)
frames = Frames()

def merge_and_publish():
    if hasattr(frames, 'gps_model') and hasattr(frames, 'lidar_model'):
        result = merge_frames([frames.gps_model, frames.lidar_model], [1.0, 1.0])
        result_msg = bridge.cv2_to_imgmsg(result)
        world_pub.publish(result_msg)


def set_gps_model(gps):
    try:
        frames.gps_model = bridge.imgmsg_to_cv2(gps, desired_encoding='mono8')
        merge_and_publish()
    except CvBridgeError as e:
        rospy.loginfo("Error converting gps model to cv2")

def set_lidar_model(lidar):
    try:
        frames.lidar_model = bridge.imgmsg_to_cv2(lidar, desired_encoding='mono8')
        merge_and_publish()
    except CvBridgeError as e:
        rospy.loginfo("Error converting lidar model to cv2")

def frame_merger_node():
    rospy.init_node('frame_merger', anonymous=True)
    rospy.Subscriber('gps_model', Image, set_gps_model)
    rospy.Subscriber('lidar_model', Image, set_lidar_model)
    rospy.spin()

def merge_frames(frames, weights):
    """
    Computes a weighted average of the given frames.

    Args:
        frames: a list of cv2 matrices to be merged
        weights: the weight each frame should have in the output

    Returns:
        A merged version of the frames
    """
    total_weight = sum(weights)

    height, width, channels = frames[0].shape
    merged = np.zeros((height, width, channels), np.uint8)

    for frame, weight in zip(frames, weights):
        alpha = (weight / total_weight)

        merged += frame * alpha

    return merged

if __name__ == '__main__': frame_merger_node()
