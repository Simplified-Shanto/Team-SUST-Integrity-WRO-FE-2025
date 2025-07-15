import numpy as np
import cv2

# --- Configuration ---
CAMERA_INDEX = 2  # Select which cam will be used
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MIN_OBJECT_AREA = 500  # Minimum contour area to consider an object (adjust as needed)
# Object dimensions and camera focal length for distance estimation (in cm and px)
# KNOWN_WIDTH_CM should correspond to the 'w' (width)  of the object
# Here we are using teh WRO future engineers game obstacle as sample object
KNOWN_WIDTH_CM = 5.0  # Object's physical width
KNOWN_DISTANCE_CM = 20

# --- Initialize webcam ---
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print(f"Error: Could not open video stream from camera index {CAMERA_INDEX}.")
    print(
        "Please check if the camera is connected and not in use by another application."
    )
    exit()


# Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

def nothing(x):
    pass

# --- Create General Trackbar Window
cv2.namedWindow("HSV Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Trackbars", 600, 300)
cv2.waitKey(100)  # Wait 100ms for the window to draw

# try:
#     cv2.moveWindow("HSV Trackbars", 10, 10)  # Move to top-left corner
#     cv2.moveWindow("Original Frame", FRAME_WIDTH + 30, 10)  # Position original frame
#     cv2.moveWindow("General Mask", 10, 400)  # Position mask below trackbars
#     cv2.moveWindow(
#         "General Result", FRAME_WIDTH + 30, 400
#     )  # Position result next to mask
#     cv2.moveWindow(
#         "Red/Blue Detection", (FRAME_WIDTH * 2) + 50, 10
#     )  # New window for Red/Blue boxes
# except cv2.error as e:
#     print(f"Could not move window. Error: {e}")

# Create trackbars for general HSV lower and upper bounds
#                                          Intial value of trackbar    Highest value of the trackbar
# Blue color range; We are using a WRO FE blue obstacle
# BLUE_LOWER = np.array([100, 150, 50])
# BLUE_UPPER = np.array([140, 255, 255])
cv2.createTrackbar("L - H", "HSV Trackbars",          100,                           179,             nothing)
cv2.createTrackbar("L - S", "HSV Trackbars", 150, 255, nothing)
cv2.createTrackbar("L - V", "HSV Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - H", "HSV Trackbars", 140, 179, nothing)
cv2.createTrackbar("U - S", "HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "HSV Trackbars", 255, 255, nothing)

def calculate_focalLength(perceived_dimension_px): 
    return round((KNOWN_DISTANCE_CM * perceived_dimension_px)/KNOWN_WIDTH_CM)


# --- Video Processing Loop ---
while True:
    ret, frame = cap.read()
    cv2.imshow("RawFrame", frame)

    if not ret:
        print("Error: Failed to grab frame. Exiting...")
        break

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    detection_frame = frame.copy()

    # Converting to HSV color 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Get Trackbar Positions for general HSV tuning ---
    l_h = cv2.getTrackbarPos("L - H", "HSV Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "HSV Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "HSV Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "HSV Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "HSV Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "HSV Trackbars")

    # Define bounds for the general trackbar mask
    lower_bound_blue = np.array([l_h, l_s, l_v])
    upper_bound_blue = np.array([u_h, u_s, u_v])

    # # Create mask and result for the general trackbar window
    # mask_general = cv2.inRange(hsv, lower_bound_general, upper_bound_general)
    # result_general = cv2.bitwise_and(frame, frame, mask=mask_general)

    blue_mask = cv2.inRange(hsv, lower_bound_blue, upper_bound_blue)
    # blue_image = cv2.bitwise_and(frame, frame, mask = blue_mask)
    # contours_blue, _ = cv2.findContours(
    #     blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
    # )

    #cv2.imshow("blue_mask", blue_mask)