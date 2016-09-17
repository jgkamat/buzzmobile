import numpy as np
import cv2

image = cv2.imread('rawImage.png')
mask = np.zeros(image.shape[:2], np.uint8)
bgdModel = np.zeros((1, 65), np.float64)
fgdModel = np.zeros((1, 65), np.float64)

manualMask = cv2.resize(cv2.imread('result.png', 0), image.shape[:2][::-1], 0, 0, cv2.INTER_NEAREST)

# Map mask to {0, 1}
mask[manualMask < 127] = 0
mask[manualMask >= 127] = 1

mask, bgdModel, fgdModel = cv2.grabCut(image, mask, None, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_MASK)
mask, bgdModel, fgdModel = cv2.grabCut(image, mask, None, bgdModel, fgdModel, 5, cv2.GC_EVAL)

mask = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
image = image * mask[:, :, np.newaxis]

cv2.imwrite("masked.png", image)
