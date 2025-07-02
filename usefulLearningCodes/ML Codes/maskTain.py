import time
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

def nothing(x):
    pass

cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars", 600, 300)

cv2.waitKey(100) # Wait 100ms for the window to draw

try:
    cv2.moveWindow("Trackbars", 10, 10) # Move to top-left corner
    cv2.moveWindow("Original Frame", 620, 10) # Position original frame
    cv2.moveWindow("Mask", 10, 400) # Position mask below trackbars
    cv2.moveWindow("Result", 620, 400) # Position result next to mask
except cv2.error as e:
    print(f"Could not move window. Error: {e}")

# Create trackbars for HSV lower and upper bounds
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# --- Frame rate control variables ---
target_fps = 30.0  # Set Camera FPS
frame_delay = int(1000 / target_fps) # Milliseconds to wait per frame

prev_frame_time = 0
new_frame_time = 0

# --- Vide Randering ---
while True:
    new_frame_time = time.time()
    # Calculate actual FPS for debugging 
    # fps = 1 / (new_frame_time - prev_frame_time)
    # prev_frame_time = new_frame_time
    # print(f"FPS: {fps:.2f}")

    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to grab frame.")
        break

    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original Frame", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    # --- Dynamic waitKey to control FPS ---
    # Calculate the time taken for processing the current frame
    processing_time = (time.time() - new_frame_time) * 1000 

    # Calculate the remaining time to wait to hit target_fps
    wait_time = max(1, int(frame_delay - processing_time)) 

    key = cv2.waitKey(wait_time) & 0xFF # Use the calculated wait_time
    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()