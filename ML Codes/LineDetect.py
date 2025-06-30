import cv2
import numpy as np

# Start webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Blue color range
    lower_blue = np.array([100, 100, 50])
    upper_blue = np.array([140, 255, 255])

    # Mask and contours
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    total_lines = 0

    for contour in contours:
        if cv2.contourArea(contour) > 500:
            total_lines += 1
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Print total line count
    print(f"Total Lines: {total_lines}")

    # Show frame
    cv2.imshow("Blue Line Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
