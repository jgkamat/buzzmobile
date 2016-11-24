#!/usr/bin/env python
"""frame_merger: node that merges n frames into a world_model

Subscribes:
    lidar_model Image model of obstables from lidar
    gps_model Image model of immediate path in route
Publishes:
    world_model Image the weighted average of the individual frames/models
"""

import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


g = {} # globals
g['lidar_model'] = g['gps_model'] = None
bridge = CvBridge()
pub = rospy.Publisher('world_model', Image, queue_size=1)


def merge_and_publish():
    """If possible, merges the current frames and publishes a world_model."""
    if g['gps_model'] is not None and g['lidar_model'] is not None:
        result = merge_frames([g['gps_model'], g['lidar_model']], [1.0, 1.0])
        result_msg = bridge.cv2_to_imgmsg(result, 'mono8')
        pub.publish(result_msg)

def set_gps_model(gps):
    """Updates current gps_model and publishes merged model."""
    try:
        g['gps_model'] = np.squeeze(
                bridge.imgmsg_to_cv2(gps, desired_encoding='mono8'))
        merge_and_publish()
    except CvBridgeError:
        rospy.loginfo("Error converting gps model to cv2")
        raise

def set_lidar_model(lidar):
    """Updates current lidar_model and publishes merged model."""
    try:
        g['lidar_model'] = np.squeeze(
                bridge.imgmsg_to_cv2(lidar, desired_encoding='mono8'))
        merge_and_publish()
    except CvBridgeError:
        rospy.loginfo("Error converting lidar model to cv2")
        raise

def merge_frames(frames, weights):
    """Computes a weighted average of the given frames.

    Args:
        frames: a list of cv2 matrices to be merged
        weights: the weight each frame should have in the output
    Returns:
        A merged version of the frames
    """
    total_weight = sum(weights)

    height, width = frames[0].shape
    merged = np.zeros((height, width), np.uint8)

    for frame, weight in zip(frames, weights):
        alpha = (weight / total_weight)
        merged += (frame * alpha).astype(np.uint8)

    return merged

def frame_merger_node():
    """Initializes frame_merger node."""
    rospy.init_node('frame_merger', anonymous=True)
    rospy.Subscriber('gps_model', Image, set_gps_model)
    rospy.Subscriber('lidar_model', Image, set_lidar_model)
    rospy.spin()

if __name__ == '__main__': frame_merger_node()
