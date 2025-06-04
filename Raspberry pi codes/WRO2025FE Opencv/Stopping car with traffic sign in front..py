#Objective: During the lap, the car will stop whenever it faces the Red traffic sign on it's way
# and if we exchange that traffic sign with a green one, the car will again resume it's mission

red_lower = np.array([])
red_upper = np.array([])

import cv2 as cv
import numpy as np
import time


frameWidth = 640
frameHeight = 480
cap = cv.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, 150) # The brightness value of the camera


while True:
    success, img = cap.read()

    cv.imshow("Image", img)

    if (cv.waitKey(1) & 0xFF) == ord('q'):
        break


