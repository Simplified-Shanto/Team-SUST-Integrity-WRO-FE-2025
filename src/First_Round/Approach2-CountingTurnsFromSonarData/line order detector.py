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

blue_lower = np.array([ ])
blue_upper = np.array([ ])

orange_lower = np.array([])
orange_upper = np.array([])

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #cv2.imshow("Original", frame)
    frame = frame[100:400, 150:590] #cropping the image to extract only useful part
    cv2.imshow("Cropped", frame)
    

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
