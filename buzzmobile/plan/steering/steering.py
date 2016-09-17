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

def project_tentacle(x_0, y_0, heading, steering_angle, travel_distance,
        num_points, wheel_base=3):
    """
    Returns a list of expected positions (in pixel coordinates),
    starting at (x, y), given constant steering_angle (in radians) and
    travel_distance (in meters). Heading is angle at which the car is facing.
    """
    if steering_angle == 0:
        y = y_0 + (PIXELS_PER_METER * travel_distance)
        if num_points == 0:
            return [(int(round(x_0)), int(round(y)))]
        return [(int(round(x_0)), int(round(y)))] + project_tentacle(x_0, y,
                heading, steering_angle, travel_distance,
                num_points - 1, wheel_base)

    radius = turning_radius(steering_angle, wheel_base)
    travel_angle = travel_distance / radius + heading

    x = x_0 + PIXELS_PER_METER * radius * (
            np.cos(heading) - np.cos(travel_angle)) * np.sign(steering_angle)
    y = y_0 + PIXELS_PER_METER * radius * (
            np.sin(travel_angle) - np.sin(heading))

    if num_points == 0:
        return [(int(round(x)), int(round(y)))]
    return [(int(round(x)), int(round(y)))] + project_tentacle(x, y,
                travel_angle, steering_angle, travel_distance,
                num_points - 1, wheel_base)


if __name__ == '__main__':
    np.set_printoptions(linewidth=200)

    height = width = 500

    frame1 = np.zeros((height, width), np.uint8)
    cv2.line(frame1, (width//2, height), (width//2, height//2), 255, width//6)
    cv2.line(frame1, (width//2, height//2), (width, height//2), 255, width//6)

    frame2 = np.zeros((height, width), np.uint8)
    cv2.circle(frame2, (width//2, height), int(width//2.5), 255, thickness=-1)

    result_frame = merge_frames([frame1, frame2], [1.0, 1.0])

    max_angle = 1.5
    travel_distance = 1
    # angles = np.linspace(0.0, max_angle, max_angle * 100)
    # colors = cm.rainbow(np.linspace(0, 1, len(angles)))

    # for angle, c in zip(angles, colors):
        # pos_points = project_tentacle(0, 0, 0, angle, travel_distance, 20)
        # neg_points = project_tentacle(0, 0, 0, -angle, travel_distance, 20)

        # plt.scatter(*zip(*pos_points), color=c)
        # plt.scatter(*zip(*neg_points), color=c)

    points = project_tentacle(width//2, height, -np.pi, 0.2, travel_distance, 20)
    # plt.scatter(*zip(*points))
    # plt.show()

    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(result_frame, pt1, pt2, [0, 150, 136], 4)

    cv2.imshow('result_frame', result_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
