#!/usr/bin/env python3

import time
import cv2
import numpy as np
import serial
from picamera2 import Picamera2



ser = serial.Serial("/dev/esp32_serial", 115200, timeout = 1)
time.sleep(2)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

blue_lower = np.array([71, 203 , 0 ])
blue_upper = np.array([179, 255, 255 ])

orange_lower = np.array([0, 127, 163 ])
orange_upper = np.array([47, 255, 255 ])

thresholdArea = 1000

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #cv2.imshow("Original", frame)
    frame = frame[100:400, 150:590] #cropping the image to extract only useful part
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
    mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
    
    blue_contours = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    orange_contours = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #This basically measures, which color of threshold area do we get first
    for cnt in blue_contours: 
        if cv2.contourArea(cnt) > thresholdArea:
            message = 'b;'
            ser.write(message).encode('utf-8')
            ser.close()
            break

    for cnt in orange_contours: 
        if cv2.contourArea(cnt) > thresholdArea:
            message = 'o;'
            ser.write(message).encode('utf-8')
            ser.close()
            break

     
    cv2.imshow("blue_mask", mask_blue)
    cv2.imshow("orange_mask", mask_orange)
    

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
