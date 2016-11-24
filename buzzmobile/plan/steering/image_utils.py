"""Image utils for steering node."""

import colorsys
import cv2
import numpy as np


def score_to_color(score):
    """Converts a score between 0 and 1 to RGB that is visible in greyscale."""
    score_to_h = lambda x: ((231 - 4) * x + 4) / 255
    score_to_s = lambda x: ((65.2 - 77.4) * x + 77.4) / 255
    score_to_v = lambda x: ((71 - 95.7) * x + 95.7) / 255

    return [255 * c for c in colorsys.hsv_to_rgb(
        score_to_h(score), score_to_s(score), score_to_v(score))]

def draw_score(x, y, score, frame):
    """Draws text with score at (x, y) in frame."""
    cv2.putText(frame, '{:.5}'.format(str(score)), (x, y),
            cv2.FONT_HERSHEY_PLAIN, 1, score_to_color(1.0))

def draw_points(points, color_frame, color, thickness=None):
    """Draws points as a line in color_frame."""
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(color_frame, pt1, pt2, color,
                thickness if thickness is not None else 1)

def draw_pose_viz(pose, frame, points, braking_distance_m):
    """Draws path and brake vizualization in image."""
    width, height = frame.shape

    if pose.brake:
        cv2.circle(frame, (width//2, height),
                int(braking_distance_m), score_to_color(1.0), 2)
        draw_points(points, frame, score_to_color(0.0), thickness=1)
    else:
        cv2.circle(frame, (width//2, height),
                int(braking_distance_m), score_to_color(0.0), 1)
        draw_points(points, frame, score_to_color(1.0), thickness=2)

def create_tentacle_mask(points, height, width, buzzmobile_width, pixels_per_m):
    """Creates a mask of a tentacle by drawing the points as a line."""
    tentacle_mask = np.zeros((height, width), np.uint8)
    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i+1]
        cv2.line(tentacle_mask, pt1, pt2, [255, 255, 255],
                int(buzzmobile_width * pixels_per_m))
    return tentacle_mask
