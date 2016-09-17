from __future__ import division
import cv2
import colorsys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm


# TODO: MODIFY THESE PARAMS
PIXELS_PER_METER = 15 # number of pixels per meter in each frame
HEIGHT = WIDTH = 500 # height and width of frames in pixels
MAX_ANGLE = 1.5 # max angle steering can happen in radians
TRAVEL_DISTANCE = 0.3 # travel distance between ack steps in meters
NUM_POINTS = 50 # number of points per tentacle
WHEEL_BASE = 3 # distance between front and back wheel axels in meters

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

def score_tentacle(points, frame):
    """
    Returns the score per tentacle by masking the frame with the tentacle's
    points, then summing the values of the result and normalizing by the sum of
    the values of the mask.
    """
    tentacle_mask = np.zeros((HEIGHT, WIDTH), np.uint8)
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(tentacle_mask, pt1, pt2, [255, 255, 255], 1)

    normalizing_factor = sum(sum(tentacle_mask))
    if normalizing_factor == 0.0:
        return 0.0

    tentacle_score_image = cv2.bitwise_and(frame, frame, mask=tentacle_mask)
    tentacle_score = sum(sum(tentacle_score_image)) / normalizing_factor
    return tentacle_score

def pick_tentacle(x_0, y_0, frame):
    angles = np.linspace(0.0, MAX_ANGLE, MAX_ANGLE * 100)
    color_frame = cv2.cvtColor(frame, cv2.cv.CV_GRAY2RGB)

    best_score = -1
    best_points = []

    for angle in angles:
        if angle == 0:
            points = project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS)

            score = score_tentacle(points, frame)
            color = score_to_color(score)
            draw_points(points, color_frame, color)

            print score, '->', color
            if score > best_score:
                best_score = score
                best_points = points
        else:
            pos_points = project_tentacle(x_0, y_0, -np.pi, angle, NUM_POINTS)
            neg_points = project_tentacle(x_0, y_0, -np.pi, -angle, NUM_POINTS)

            pos_score = score_tentacle(pos_points, frame)
            pos_color = score_to_color(pos_score)
            draw_points(pos_points, color_frame, pos_color)

            neg_score = score_tentacle(neg_points, frame)
            neg_color = score_to_color(neg_score)
            draw_points(neg_points, color_frame, neg_color)

            print pos_score, '->', pos_color
            print neg_score, '->', neg_color
            if pos_score > best_score:
                best_score = pos_score
                best_points = pos_points

            if neg_score > best_score:
                best_score = neg_score
                best_points = neg_points

    draw_points(best_points, color_frame, (255, 0, 0))

    cv2.imshow('all tentacles', color_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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


if __name__ == '__main__':
    def merge_frames(frames, weights):
        total_weight = sum(weights)

        height, width = frames[0].shape
        merged = np.zeros((height, width), np.uint8)

        for frame, weight in zip(frames, weights):
            alpha = (weight / total_weight)
            merged = merged + (frame * alpha).astype(np.uint8)

        return merged

    frame1 = np.zeros((HEIGHT, WIDTH), np.uint8)
    cv2.line(frame1, (WIDTH//2, HEIGHT), (WIDTH//2, HEIGHT//2), 255, WIDTH//6)
    cv2.line(frame1, (WIDTH//2, HEIGHT//2), (WIDTH, HEIGHT//2), 255, WIDTH//6)

    frame2 = np.zeros((HEIGHT, WIDTH), np.uint8)
    cv2.circle(frame2, (WIDTH//2, HEIGHT), int(WIDTH//2.5), 255, thickness=-1)

    result_frame = merge_frames([frame1, frame2], [1.0, 1.0])

    pick_tentacle(WIDTH//2, HEIGHT, result_frame)
