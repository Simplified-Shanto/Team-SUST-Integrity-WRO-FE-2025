import time
import json
import queue
import threading
import serial
import numpy as np
import cv2

from picamera2 import Picamera2

# Serial setup (change port if needed)
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1.0)
time.sleep(2)
serial_queue = queue.Queue()

# Serial writer thread
def serial_writer():
    while True:
        data = serial_queue.get()
        if data is None:
            break
        try:
            ser.write((data + "\n").encode("utf-8"))
            ser.flush()
            print("Sent:", data)
            time.sleep(0.01)
        except Exception as e:
            print("Serial write error:", e)

thread = threading.Thread(target=serial_writer)
thread.daemon = True
thread.start()

# Setup Pi Camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(1)

# Cooldowns and counters
cooldown = 0.5
last_blue_time = 0
last_orange_time = 0
last_send_time = 0
send_interval = 0.1

blue_count = 0
orange_count = 0
last_sent_data = ""

# HSV color ranges
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([140, 255, 255])
lower_orange = np.array([5, 100, 100])
upper_orange = np.array([25, 255, 255])

# Crossing band
cross_y_min = 300
cross_y_max = 400

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    now = time.time()

    # Blue detection
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_blue:
        if cv2.contourArea(cnt) > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cy = y + h // 2
            if cross_y_min < cy < cross_y_max and (now - last_blue_time > cooldown):
                blue_count += 1
                last_blue_time = now
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Orange detection
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_orange:
        if cv2.contourArea(cnt) > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cy = y + h // 2
            if cross_y_min < cy < cross_y_max and (now - last_orange_time > cooldown):
                orange_count += 1
                last_orange_time = now
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)

    # Send data
    line_info = {
        "BlueLinesPassed": blue_count,
        "OrangeLinesPassed": orange_count,
        "TotalLinesPassed": blue_count + orange_count,
    }
    json_data = json.dumps(line_info)

    if json_data != last_sent_data or (now - last_send_time > send_interval):
        serial_queue.put(json_data)
        last_sent_data = json_data
        last_send_time = now

    # Display
    cv2.line(frame, (0, cross_y_min), (frame.shape[1], cross_y_min), (255, 255, 255), 1)
    cv2.line(frame, (0, cross_y_max), (frame.shape[1], cross_y_max), (255, 255, 255), 1)
    cv2.putText(frame, f"Blue: {blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, f"Orange: {orange_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 2)
    cv2.imshow("Line Counter", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
serial_queue.put(None)
thread.join()
ser.close()
cv2.destroyAllWindows()
