import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Global Variables
bridge = CvBridge()
lidar_model = None
gps_model = None
world_pub = rospy.Publisher('world_model', Image, queue_size=0)

def merge_and_publish():
    result = merge_frames([gps_model, lidar_model], [1.0, 1.0])
    result_msg = bridge.cv2_to_imgmsg(result)

    world_pub.publish(result_msg)
    

def set_gps_model(gps):
    try:
        gps_model = bridge.imgmsg_to_cv2(gps)
        merge_and_publish()
    except CvBridgeError as e:
        rospy.loginfo("Error converting gps model to cv2")

def set_lidar_model(lidar):
    try:
        lidar_model = bridge.imgmsg_to_cv2(lidar)
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

    height, width = frames[0].shape
    merged = np.zeros((height, width), np.uint8)

    for frame, weight in zip(frames, weights):
        alpha = (weight / total_weight)
        merged = merged + (frame * alpha).astype(np.uint8)

    return merged

if __name__ == '__main__':
    frame_merger_node()
    # np.set_printoptions(linewidth=200)

    # height = width = 500

    # frame1 = np.zeros((height, width), np.uint8)
    # cv2.line(frame1, (width/2, height), (width/2, height/2), 255, width/6)
    # cv2.line(frame1, (width/2, height/2), (width, height/2), 255, width/6)

    # frame2 = np.zeros((height, width), np.uint8)
    # cv2.circle(frame2, (width/2, height), int(width//2.5), 255, thickness=-1)

    # result_frame = merge_frames([frame1, frame2], [1.0, 1.0])

    # cv2.imshow('frame1', frame1)
    # cv2.imshow('frame2', frame2)
    # cv2.imshow('result_frame', result_frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
