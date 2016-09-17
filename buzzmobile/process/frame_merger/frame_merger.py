import cv2
import numpy as np


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
    np.set_printoptions(linewidth=200)

    height = width = 500

    frame1 = np.zeros((height, width), np.uint8)
    cv2.line(frame1, (width/2, height), (width/2, height/2), 255, width/6)
    cv2.line(frame1, (width/2, height/2), (width, height/2), 255, width/6)

    frame2 = np.zeros((height, width), np.uint8)
    cv2.circle(frame2, (width/2, height), int(width//2.5), 255, thickness=-1)

    result_frame = merge_frames([frame1, frame2], [1.0, 1.0])

    cv2.imshow('frame1', frame1)
    cv2.imshow('frame2', frame2)
    cv2.imshow('result_frame', result_frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
