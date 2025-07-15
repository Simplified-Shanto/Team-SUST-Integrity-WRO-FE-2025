

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

    blur_blue = cv2.GaussianBlur(blue_mask, (5, 5), 0)
    blur_orange = cv2.GaussianBlur(orange_mask, (5, 5), 0)

    edges_blue = cv2.Canny(blur_blue, 50, 150)
    edges_orange = cv2.Canny(blur_orange, 50, 150)

    lines_blue = cv2.HoughLinesP(edges_blue, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
    lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)

    line_count_blue = 0
    line_count_orange = 0

    if lines_blue is not None:
        for line in lines_blue:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            line_count_blue += 1

    if lines_orange is not None:
        for line in lines_orange:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 140, 255), 2)
            line_count_orange += 1

    cv2.putText(frame, f"Blue Lines: {line_count_blue}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    cv2.putText(frame, f"Orange Lines: {line_count_orange}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 2)

    cv2.imshow("Line Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
