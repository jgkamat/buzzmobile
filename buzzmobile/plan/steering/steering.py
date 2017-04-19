#!/usr/bin/env python
"""steering: calculates whether to brake, or where to turn, and publishes it.

Uses ackerman's steering to determine where to turn.

Subscribes:
    world_model Image calculated model of movability around the car
    lidar_model Image model of drivable (obstacle-free) area in front of car
Publishes:
    auto_car_pose CarPose with command (whether to brake, where to turn)
    tentacle_frame Image visualization of CarPose, for mission_control
"""

from __future__ import division
# TODO(irapha): remove this line when #126 is fixed.

import cv2
import numpy as np
import rospy

from buzzmobile.msg import CarPose
from cv_bridge import CvBridge
from image_utils import draw_pose_viz, create_tentacle_mask
from sensor_msgs.msg import Image


g = {} # globals
g['lidar_model'] = None
bridge = CvBridge()
pub = rospy.Publisher('auto_car_pose', CarPose, queue_size=1)
TENTACLE_PUB = rospy.Publisher('tentacle_frame', Image, queue_size=1)

PIXELS_PER_M = rospy.get_param('pixels_per_m')
HEIGHT = rospy.get_param('image_height')
WIDTH = rospy.get_param('image_width')
MAX_ANGLE = rospy.get_param('max_steering_angle')
TRAVEL_DISTANCE = rospy.get_param('travel_distance')
NUM_POINTS = rospy.get_param('num_points_in_tentacle')
WHEEL_BASE = rospy.get_param('wheel_base')
ANGLE_MULTIPLIER = rospy.get_param('angle_multiplier')
BRAKING_DISTANCE = rospy.get_param('braking_distance')
THRESHHOLD = rospy.get_param('braking_score_threshhold')
BUZZMOBILE_WIDTH = rospy.get_param('buzzmobile_width')
MAX_SPEED = rospy.get_param('max_speed', 2.0) #TODO don't be a murder machine

IMMEDIATE_FUTURE_MASK = np.zeros((HEIGHT, WIDTH), np.uint8)
cv2.circle(IMMEDIATE_FUTURE_MASK, (WIDTH//2, HEIGHT),
        int(BRAKING_DISTANCE * PIXELS_PER_M), [255, 255, 255], -1)


def steer(ros_world_model):
    """Performs best path and brake calculations, and publishes a car_pose."""
    # convert RosImage to cv2
    world_frame = np.squeeze(bridge.imgmsg_to_cv2(ros_world_model, 'mono8'))

    # pick tentacle
    height, width = world_frame.shape
    points, angle = pick_tentacle(width//2, height, world_frame)

    pose = CarPose()
    pose.mode = 'auto'

    # check our path for obstacles
    if should_brake(points, g['lidar_model']):
        pose.brake = True
    else:
        pose.angle = angle
        pose.velocity = MAX_SPEED

    # publish carpose
    pub.publish(pose)

    # publish drawn tentacle
    draw_pose_viz(pose, world_frame, points, (BRAKING_DISTANCE * PIXELS_PER_M))
    tentacle_frame = bridge.cv2_to_imgmsg(world_frame)
    TENTACLE_PUB.publish(tentacle_frame)

def set_lidar_model(new_lidar_model):
    """Updates lidar_model."""
    lidar_model = bridge.imgmsg_to_cv2(new_lidar_model, 'mono8')
    g['lidar_model'] = lidar_model

def should_brake(points, lidar_model):
    """Returns whether we should brake, given best path and obstacle model."""
    if lidar_model is None:
        rospy.loginfo('braking because no lidar image received.')
        return True

    tentacle_mask = create_tentacle_mask(points, HEIGHT, WIDTH,
            BUZZMOBILE_WIDTH, PIXELS_PER_M)
    immediate_path_mask = cv2.bitwise_and(IMMEDIATE_FUTURE_MASK, tentacle_mask)
    lidar_model_path = cv2.bitwise_and(lidar_model, lidar_model,
                                       mask=immediate_path_mask)

    # must use np.sum. cv2 uses ints to store pixels, so sum(sum()) overflows.
    score = (np.sum(np.sum(lidar_model_path)) /
             float(np.sum(np.sum(immediate_path_mask))))
    return score < THRESHHOLD

def turning_radius(steering_angle):
    """Returns turning radius given steering angle (rad)."""
    if steering_angle == 0:
        return float('inf')
    return abs(WHEEL_BASE / np.tan(steering_angle))

def ackerman_step(x_0, y_0, heading, steering_angle):
    """Single ackerman's steering pose estimation step.
    Returns the final x, y, and the new heading.
    """
    if steering_angle == 0:
        y = y_0 + (PIXELS_PER_M * TRAVEL_DISTANCE) * (1 if heading == 0 else -1)
        return x_0, y, heading

    radius = turning_radius(steering_angle)
    travel_angle = TRAVEL_DISTANCE / radius + heading

    x = x_0 + PIXELS_PER_M * radius * (
            np.cos(heading) - np.cos(travel_angle)) * np.sign(steering_angle)
    y = y_0 + PIXELS_PER_M * radius * (
            np.sin(travel_angle) - np.sin(heading))

    return x, y, travel_angle

def project_tentacle(x_0, y_0, heading, steering_angle,
        num_points):
    """Returns expected future positions using ackerman's steering.

    Recursively computes expected positions (in pixel coordinates),
    starting at (x, y), given constant steering_angle (in radians) and
    TRAVEL_DISTANCE (in meters). Heading is angle at which the car is facing.
    """
    x, y, heading = ackerman_step(x_0, y_0, heading, steering_angle)
    if num_points == 0:
        return [(int(round(x)), int(round(y)))]

    return [(int(round(x)), int(round(y)))] + project_tentacle(x, y,
            heading, steering_angle, num_points - 1)

def score_tentacle(points, frame):
    """Returns a tentacle's score from 0 to 1.

    Calculated by masking the frame with the tentacle's
    points, then adding the values of the result and normalizing by the sum of
    the values of the mask.
    """
    tentacle_mask = create_tentacle_mask(points, HEIGHT, WIDTH,
            BUZZMOBILE_WIDTH, PIXELS_PER_M)

    # must use np.sum. cv2 uses ints to store pixels, so sum(sum()) overflows.
    normalizing_factor = np.sum(np.sum(tentacle_mask))
    if normalizing_factor == 0.0:
        rospy.logerr('Mask has no pixels')
        return 0.0

    tentacle_score_image = cv2.bitwise_and(frame, frame, mask=tentacle_mask)
    # must use np.sum. cv2 uses ints to store pixels, so sum(sum()) overflows.
    tentacle_score = np.sum(np.sum(tentacle_score_image)) / normalizing_factor
    return tentacle_score

def pick_tentacle(x_0, y_0, frame):
    """Returns the tentacle with the highest score provided by score_tentacle.
    Breaks ties by picking the tentacle with the smallest magnitude angle
    """
    angles = np.linspace(0.0, MAX_ANGLE, MAX_ANGLE * ANGLE_MULTIPLIER)

    best_score = -1
    best_points = []
    best_angle = 0

    for angle in angles:
        if angle == 0:
            branches = [project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS)]
        else:
            branches = [project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS),
                    project_tentacle(x_0, y_0, -np.pi, -angle, NUM_POINTS)]

        for points in branches:
            score = score_tentacle(points, frame)

            if score > best_score or (score == best_score
                    and abs(angle) < abs(best_angle)):
                best_score = score
                best_points = points
                best_angle = angle

    return best_points, best_angle

def steering_node():
    """Initializes steering node."""
    rospy.init_node('steering', anonymous=True)
    rospy.Subscriber('world_model', Image, steer)
    rospy.Subscriber('lidar_model', Image, set_lidar_model)
    rospy.spin()

if __name__ == '__main__': steering_node()
