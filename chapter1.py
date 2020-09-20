import cv2
import numpy as np

print("Package imported")

img = cv2.imread("Resources/lena.png")
# cap = cv2.VideoCapture("Resources/test_video.mp4")
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
cap.set(10, 10)  # brightness

# cv2.imshow("Output", img)
# cv2.waitKey(0)

while True:
    success, img_cap = cap.read()
    cv2.imshow("Video", img_cap)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
