import cv2
import numpy as np
import time
import json

cap = cv2.VideoCapture(0)

# Cooldown timers
last_blue_time = 0
last_orange_time = 0
cooldown = 1.5  # seconds

# Count variables
blue_count = 0
orange_count = 0

# Define color ranges in HSV
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([140, 255, 255])

lower_orange = np.array([5, 100, 100])
upper_orange = np.array([25, 255, 255])

# Define Y-crossing band
cross_y_min = 300
cross_y_max = 400

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Masks
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

    now = time.time()

    # Blue Line Detection 
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_blue:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cy = y + h // 2
            if cross_y_min < cy < cross_y_max:
                if now - last_blue_time > cooldown:
                    blue_count += 1
                    last_blue_time = now
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # Orange Line Detection 
    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_orange:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cy = y + h // 2
            if cross_y_min < cy < cross_y_max:
                if now - last_orange_time > cooldown:
                    orange_count += 1
                    last_orange_time = now
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)

    # Output
    line_info = {
        "BlueLinesPassed": blue_count,
        "OrangeLinesPassed": orange_count,
        "TotalLinesPassed": blue_count + orange_count
    }

    print(json.dumps(line_info, indent=2))

    # Draw crossing band
    cv2.line(frame, (0, cross_y_min), (frame.shape[1], cross_y_min), (255, 255, 255), 1)
    cv2.line(frame, (0, cross_y_max), (frame.shape[1], cross_y_max), (255, 255, 255), 1)

    # Display counts 
    cv2.putText(frame, f"Blue Passed: {blue_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, f"Orange Passed: {orange_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 2)

    # Show the result
    cv2.imshow("Line Passing Counter", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
