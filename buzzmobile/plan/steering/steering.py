#!/usr/bin/env python
from __future__ import division

import colorsys
import cv2
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import rospy

from buzzmobile.msg import CarPose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


### GLOBAL VARS ###
bridge = CvBridge()
pose_pub = rospy.Publisher('car_pose', CarPose, queue_size=0)
tentacle_pub = rospy.Publisher('tentacle_frame', Image, queue_size=0)

PIXELS_PER_METER = rospy.get_param('pixels_per_m')
HEIGHT = rospy.get_param('image_height')
WIDTH = rospy.get_param('image_width')
MAX_ANGLE = rospy.get_param('max_steering_angle')
TRAVEL_DISTANCE = rospy.get_param('travel_distance')
NUM_POINTS = rospy.get_param('num_points_in_tentacle')
WHEEL_BASE = rospy.get_param('wheel_base')
ANGLE_MULTIPLIER = rospy.get_param('angle_multiplier')
BUZZMOBILE_WIDTH = rospy.get_param('buzzmobile_width')
BRAKING_DISTANCE = rospy.get_param('braking_distance')
THRESHHOLD = rospy.get_param('braking_score_threshhold')

# this gets modified on steering_node init
immediate_future_mask = np.zeros((HEIGHT, WIDTH), np.uint8)
saved_models = {'lidar_model': None}


def steering_node():
    create_immediate_future_mask()
    rospy.init_node('steering', anonymous=True)
    rospy.Subscriber('world_model', Image, steer)
    rospy.Subscriber('lidar_model', Image, set_lidar_model)
    rospy.spin()

def steer(ros_world_model):
    # convert RosImage to cv2
    try:
        world_frame = bridge.imgmsg_to_cv2(ros_world_model)
    except CvBridgeError:
        rospy.loginfo('Error converting world_model to cv2')

    # pick tentacle
    height, width = world_frame.shape
    points, angle = pick_tentacle(width//2, height, world_frame)

    pose = CarPose()

    # check our path for obstacles
    if should_brake(points, saved_models['lidar_model'], world_frame):
        pose.brake = True
    else:
        pose.angle = angle
        pose.velocity = 1.0

    # publish carpose
    pose_pub.publish(pose)

    # publish drawn tentacle
    if not pose.brake:
        draw_points(points, world_frame, score_to_color(1.0))
    try:
        tentacle_frame = bridge.cv2_to_imgmsg(world_frame)
        tentacle_pub.publish(tentacle_frame)
    except CvBridgeError:
        rospy.loginfo('Error converting tentacle_frame to RosImage')


def create_immediate_future_mask():
    cv2.circle(immediate_future_mask, (HEIGHT, WIDTH//2),
            int(BRAKING_DISTANCE * PIXELS_PER_METER), [255, 255, 255])

def set_lidar_model(new_lidar_model):
    try:
        lidar_model = bridge.imgmsg_to_cv2(new_lidar_model, 'mono8')
    except CvBridgeError:
        rospy.loginfo('Error converting world_model to cv2 in set_lidar_model')
    saved_models['lidar_model'] = lidar_model

def should_brake(points, lidar_model, world_frame):
    if lidar_model is None:
        rospy.loginfo('braking because no lidar image received.')
        return True

    tentacle_mask = create_tentacle_mask(points)
    immediate_path_mask = cv2.bitwise_and(lidar_model, tentacle_mask)
    world_frame_path = cv2.bitwise_and(world_frame, world_frame, mask=immediate_path_mask)

    score = sum(sum(world_frame_path)) / float(sum(sum(immediate_path_mask)))

    return True if score < THRESHHOLD else False

def turning_radius(steering_angle):
    """
    Returns turning radius given current steering angle (in radians), and
    distance between front and back axel (in meters).
    """
    if steering_angle == 0:
        return float('inf')
    return abs(WHEEL_BASE / np.tan(steering_angle))

def ackerman_step(x_0, y_0, heading, steering_angle):
    """
    Performs a single ackerman's steering pose estimation step, returning the
    final x, y, and the new heading.
    """
    if steering_angle == 0:
        y = y_0 + (PIXELS_PER_METER * TRAVEL_DISTANCE) * (1 if heading == 0.0 else -1)
        return x_0, y, heading

    radius = turning_radius(steering_angle)
    travel_angle = TRAVEL_DISTANCE / radius + heading

    x = x_0 + PIXELS_PER_METER * radius * (
            np.cos(heading) - np.cos(travel_angle)) * np.sign(steering_angle)
    y = y_0 + PIXELS_PER_METER * radius * (
            np.sin(travel_angle) - np.sin(heading))

    return x, y, travel_angle


def project_tentacle(x_0, y_0, heading, steering_angle,
        num_points):
    """
    Returns a list of expected positions (in pixel coordinates),
    starting at (x, y), given constant steering_angle (in radians) and
    TRAVEL_DISTANCE (in meters). Heading is angle at which the car is facing.
    """
    x, y, heading = ackerman_step(x_0, y_0, heading, steering_angle)
    if num_points == 0:
        return [(int(round(x)), int(round(y)))]

    return [(int(round(x)), int(round(y)))] + project_tentacle(x, y,
            heading, steering_angle, num_points - 1)

def create_tentacle_mask(points):
    tentacle_mask = np.zeros((HEIGHT, WIDTH), np.uint8)
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(tentacle_mask, pt1, pt2, [255, 255, 255], int(BUZZMOBILE_WIDTH * PIXELS_PER_METER))
    return tentacle_mask

def score_tentacle(points, frame):
    """
    Returns the score per tentacle by masking the frame with the tentacle's
    points, then summing the values of the result and normalizing by the sum of
    the values of the mask.
    """
    tentacle_mask = create_tentacle_mask(points)

    normalizing_factor = sum(sum(tentacle_mask))
    if normalizing_factor == 0.0:
        return 0.0

    tentacle_score_image = cv2.bitwise_and(frame, frame, mask=tentacle_mask)
    tentacle_score = sum(sum(tentacle_score_image)) / normalizing_factor
    return tentacle_score

def pick_tentacle(x_0, y_0, frame):
    angles = np.linspace(0.0, MAX_ANGLE, MAX_ANGLE * ANGLE_MULTIPLIER)
    color_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

    best_score = -1
    best_points = []
    best_angle = -1

    for angle in angles:
        if angle == 0:
            branches = [project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS)]
        else:
            branches = [project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS),
                    project_tentacle(x_0, y_0, -np.pi, -angle, NUM_POINTS)]

        for points in branches:
            score = score_tentacle(points, frame)

            if score > best_score:
                best_score = score
                best_points = points
                best_angle = angle

    return best_points, best_angle

def score_to_color(score):
    score_to_h = lambda x: ((231 - 4) * x + 4) / 255
    score_to_s = lambda x: ((65.2 - 77.4) * x + 77.4) / 255
    score_to_v = lambda x: ((71 - 95.7) * x + 95.7) / 255

    return [255 * c for c in colorsys.hsv_to_rgb(
        score_to_h(score), score_to_s(score), score_to_v(score))]


def draw_points(points, color_frame, color):
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(color_frame, pt1, pt2, color, 1)


if __name__ == '__main__': steering_node()
