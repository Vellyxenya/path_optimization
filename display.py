import cv2
import numpy as np

img = np.zeros((520, 840, 3), np.uint8)
print(img.shape)
height, width, channels = img.shape
img[20:(height-20), 20:(width-20)] = (255, 100, 100)
cv2.line(img, (30, 20), (30, height), (0, 0, 0))

cv2.imshow("Image", img)
cv2.waitKey(0)