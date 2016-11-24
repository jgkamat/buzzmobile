"""Creates a mask from rawImage.png."""
import numpy as np
import cv2


def main():
    """main method for grabCut."""
    image = cv2.imread('rawImage.png')
    mask = np.zeros(image.shape[:2], np.uint8)
    bgd_model = np.zeros((1, 65), np.float64)
    fgd_model = np.zeros((1, 65), np.float64)

    manual_mask = cv2.resize(cv2.imread('result.png', 0),
                            image.shape[:2][::-1],
                            0, 0, cv2.INTER_NEAREST)

    # Map mask to {0, 1}
    mask[manual_mask < 127] = 0
    mask[manual_mask >= 127] = 1

    mask, bgd_model, fgd_model = cv2.grabCut(image, mask, None, bgd_model,
                                             fgd_model, 5,
                                             cv2.GC_INIT_WITH_MASK)
    mask, bgd_model, fgd_model = cv2.grabCut(image, mask, None, bgd_model,
                                             fgd_model, 5, cv2.GC_EVAL)

    mask = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    image = image * mask[:, :, np.newaxis]

    cv2.imwrite("masked.png", image)

if __name__ == '__main__': main()
