from __future__ import division
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# TODO: REMOVE, use ROS node message.
def merge_frames(frames, weights):
    total_weight = sum(weights)

    height, width = frames[0].shape
    merged = np.zeros((height, width), np.uint8)

    for frame, weight in zip(frames, weights):
        alpha = (weight / total_weight)
        merged = merged + (frame * alpha).astype(np.uint8)

    return merged
# END REMOVE


PIXELS_PER_METER = 15 # TODO: MODIFY THIS ONCE IMAGE IS FINAL

def turning_radius(steering_angle, wheel_base=3):
    """
    Returns turning radius given current steering angle (in radians), and
    distance between front and back axel (in meters).
    """
    if steering_angle == 0:
        return float('inf')
    return abs(wheel_base / np.tan(steering_angle))

def ackerman_step(x_0, y_0, heading, steering_angle, travel_distance,
        wheel_base=3):
    """
    Performs a single ackerman's steering pose estimation step, returning the
    final x, y, and the new heading.
    """
    if steering_angle == 0:
        y = y_0 + (PIXELS_PER_METER * travel_distance)
        return x_0, y, heading

    radius = turning_radius(steering_angle, wheel_base)
    travel_angle = travel_distance / radius + heading

    x = x_0 + PIXELS_PER_METER * radius * (
            np.cos(heading) - np.cos(travel_angle)) * np.sign(steering_angle)
    y = y_0 + PIXELS_PER_METER * radius * (
            np.sin(travel_angle) - np.sin(heading))

    return x, y, travel_angle


def project_tentacle(x_0, y_0, heading, steering_angle, travel_distance,
        num_points, wheel_base=3):
    """
    Returns a list of expected positions (in pixel coordinates),
    starting at (x, y), given constant steering_angle (in radians) and
    travel_distance (in meters). Heading is angle at which the car is facing.
    """
    x, y, heading = ackerman_step(x_0, y_0, heading, steering_angle,
            travel_distance, wheel_base)
    if num_points == 0:
        return [(int(round(x)), int(round(y)))]

    return [(int(round(x)), int(round(y)))] + project_tentacle(x, y,
            heading, steering_angle, travel_distance,
            num_points - 1, wheel_base)

def score_tentacle(width, height, points, frame):
    tentacle_mask = np.zeros((height, width), np.uint8)
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(tentacle_mask, pt1, pt2, [255, 255, 255], 1)
    tentacle_score_image = cv2.bitwise_and(frame, frame, mask=tentacle_mask)
    tentacle_score = sum(sum(tentacle_score_image)) / sum(sum(tentacle_mask))
    return tentacle_score_image, tentacle_score



if __name__ == '__main__':
    np.set_printoptions(linewidth=200)

    height = width = 500

    frame1 = np.zeros((height, width), np.uint8)
    cv2.line(frame1, (width//2, height), (width//2, height//2), 255, width//6)
    cv2.line(frame1, (width//2, height//2), (width, height//2), 255, width//6)

    frame2 = np.zeros((height, width), np.uint8)
    cv2.circle(frame2, (width//2, height), int(width//2.5), 255, thickness=-1)

    result_frame = merge_frames([frame1, frame2], [1.0, 1.0])
    result_frame_color = cv2.cvtColor(result_frame, cv2.cv.CV_GRAY2RGB)

    max_angle = 1.5
    travel_distance = 0.3
    num_points = 50

    # angles = np.linspace(0.0, max_angle, max_angle * 100)
    # colors = cm.rainbow(np.linspace(0, 1, len(angles)))
    # for angle, c in zip(angles, colors):
        # pos_points = project_tentacle(0, 0, 0, angle, travel_distance, 20)
        # neg_points = project_tentacle(0, 0, 0, -angle, travel_distance, 20)

        # plt.scatter(*zip(*pos_points), color=c)
        # plt.scatter(*zip(*neg_points), color=c)
    # plt.show()

    points_1 = project_tentacle(width//2, height, -np.pi, 0.2, travel_distance, num_points)
    tentacle_score_image_1, tentacle_score_1 = score_tentacle(width, height, points_1, result_frame)
    cv2.imshow('tentacle_score_image_1', tentacle_score_image_1)
    print 'points_1 score:', tentacle_score_1

    points_2 = project_tentacle(width//2, height, -np.pi, 0.1, travel_distance, num_points)
    tentacle_score_image_2, tentacle_score_2 = score_tentacle(width, height, points_2, result_frame)
    cv2.imshow('tentacle_score_image_2', tentacle_score_image_2)
    print 'points_2 score:', tentacle_score_2

    for i in range(len(points_1) - 1):
        pt1 = points_1[i]
        pt2 = points_1[i+1]
        cv2.line(result_frame_color, pt1, pt2, [0, 150, 136], 1)

    for i in range(len(points_2) - 1):
        pt1 = points_2[i]
        pt2 = points_2[i+1]
        cv2.line(result_frame_color, pt1, pt2, [0, 150, 136], 1)

    cv2.imshow('result_frame_color', result_frame_color)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
