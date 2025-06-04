import cv2
import numpy as np

cap = cv2.VideoCapture(0)

blue_lower = np.array([103, 33, 71])
blue_upper = np.array([158, 255, 255])
orange_lower = np.array([0, 36, 130])
orange_upper = np.array([39, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)

    contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blue_line_count = 0
    orange_line_count = 0

    for cnt in contours_blue:
        x, y, w, h = cv2.boundingRect(cnt)
        if min(w, h) > 15:  # Adjust 15 based on resolution (this filters out thin junk)
            blue_line_count += 1
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    for cnt in contours_orange:
        x, y, w, h = cv2.boundingRect(cnt)
        if min(w, h) > 15:
            orange_line_count += 1
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 140, 255), 2)

    cv2.putText(frame, f"Blue Lines: {blue_line_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, f"Orange Lines: {orange_line_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 2)

    cv2.imshow("Line Detection (Thick Lines Only)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
