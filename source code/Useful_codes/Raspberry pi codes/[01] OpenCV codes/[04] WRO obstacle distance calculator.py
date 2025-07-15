import time
import numpy as np
import cv2
import json

# --- Configuration ---
CAMERA_INDEX = 0  # Select which cam will be used
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 30.0  # Cam FPS
MIN_WAIT_TIME_MS = 5  # Minimum wait time to ensure GUI responsiveness (e.g., 5ms)
MIN_OBJECT_AREA = 500  # Minimum contour area to consider an object (adjust as needed)

# Object dimensions and camera focal length for distance estimation (in cm and px)
# KNOWN_WIDTH_CM should correspond to the 'w' (width)  of the object
KNOWN_WIDTH_CM = 5.0  # Object's physical width
FOCAL_LENGTH_PX = 875.0  # Focal length in pixels 

# --- Safe Zone Configuration (in pixels) ---
SAFE_ZONE_WIDTH_PX = 100  # Width of the safe zone in the center of the frame
SAFE_ZONE_LINE_COLOR = (255, 255, 0)  # Cyan color for safe zone lines (BGR)
SAFE_ZONE_LINE_THICKNESS = 2


# --- Define HSV ranges for Red and Blue (Need tuning!) ---
# Red color mask 
RED_LOWER_1 = np.array([0, 120, 70])
RED_UPPER_1 = np.array([10, 255, 255])
RED_LOWER_2 = np.array([170, 120, 70])
RED_UPPER_2 = np.array([180, 255, 255])

# Blue color range 
BLUE_LOWER = np.array([100, 150, 50])
BLUE_UPPER = np.array([140, 255, 255])


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
cv2.namedWindow("General HSV Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("General HSV Trackbars", 600, 300)

cv2.waitKey(100)  # Wait 100ms for the window to draw

try:
    cv2.moveWindow("General HSV Trackbars", 10, 10)  # Move to top-left corner
    cv2.moveWindow("Original Frame", FRAME_WIDTH + 30, 10)  # Position original frame
    cv2.moveWindow("General Mask", 10, 400)  # Position mask below trackbars
    cv2.moveWindow(
        "General Result", FRAME_WIDTH + 30, 400
    )  # Position result next to mask
    cv2.moveWindow(
        "Red/Blue Detection", (FRAME_WIDTH * 2) + 50, 10
    )  # New window for Red/Blue boxes
except cv2.error as e:
    print(f"Could not move window. Error: {e}")

# Create trackbars for general HSV lower and upper bounds
cv2.createTrackbar("L - H", "General HSV Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "General HSV Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "General HSV Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "General HSV Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "General HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "General HSV Trackbars", 255, 255, nothing)


def estimate_distance(perceived_dimension_px):
    """Estimates distance to an object given its perceived dimension in pixels.
    Uses KNOWN_WIDTH_CM and FOCAL_LENGTH_PX from global config."""
    if perceived_dimension_px == 0:
        return 0.0
    return round((KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / perceived_dimension_px, 2)


# --- Frame rate control  ---
frame_delay_target_ms = int(1000 / TARGET_FPS)

# --- Calculate Safe Zone Coordinates for drawing and per-object calculation ---
SAFE_ZONE_CENTER_X = FRAME_WIDTH // 2
SAFE_ZONE_LEFT_EDGE_X = SAFE_ZONE_CENTER_X - (SAFE_ZONE_WIDTH_PX // 2)
SAFE_ZONE_RIGHT_EDGE_X = SAFE_ZONE_CENTER_X + (SAFE_ZONE_WIDTH_PX // 2)


# --- Video Processing Loop ---
while True:
    current_loop_start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame. Exiting...")
        break

    # # Flip horizontally -> if you are using the front facing webcam of your computer
    # frame = cv2.flip(frame, 1)

    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    detection_frame = frame.copy()

    # Converting to HSV color 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Get Trackbar Positions for general HSV tuning ---
    l_h = cv2.getTrackbarPos("L - H", "General HSV Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "General HSV Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "General HSV Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "General HSV Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "General HSV Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "General HSV Trackbars")

    # Define bounds for the general trackbar mask
    lower_bound_general = np.array([l_h, l_s, l_v])
    upper_bound_general = np.array([u_h, u_s, u_v])

    # Create mask and result for the general trackbar window
    mask_general = cv2.inRange(hsv, lower_bound_general, upper_bound_general)
    result_general = cv2.bitwise_and(frame, frame, mask=mask_general)


    # Json Output
    json_output = {
        "RedObject": {
            "IsRed": False,
            "RedCounter": 0,
            "RedDis": 0.0,  
            "RedAxis": {},  
        },
        "BlueObject": {
            "IsBlue": False,
            "BlueCounter": 0,
            "BlueDis": 0.0,  
            "BlueAxis": {},  
        },
    }

    # Lists to temporarily store data for calculating for Json
    red_distances_list = []
    blue_distances_list = []
    red_objects_details_unsorted = []
    blue_objects_details_unsorted = []

    # --- calculate SafeDis and SaDir for an object ---
    def get_safe_zone_guidance(obj_center_x):
        safe_dis_px = 0.0
        sa_dir = "center"

        if obj_center_x < SAFE_ZONE_LEFT_EDGE_X:
            safe_dis_px = SAFE_ZONE_LEFT_EDGE_X - obj_center_x
            sa_dir = "right"  # Object is to the left, robot needs to turn right to get it into zone
        elif obj_center_x > SAFE_ZONE_RIGHT_EDGE_X:
            safe_dis_px = obj_center_x - SAFE_ZONE_RIGHT_EDGE_X
            sa_dir = "left"  # Object is to the right, robot needs to turn left to get it into zone
        else:  # Object is within the safe zone
            sa_dir = "center"
            safe_dis_px = 0.0  

        return round(safe_dis_px, 2), sa_dir

    # 1. Detect Red objects
    red_mask1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
    red_mask2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    red_mask_combined = cv2.bitwise_or(red_mask1, red_mask2)

    contours_red, _ = cv2.findContours(
        red_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour_idx, contour in enumerate(contours_red):
        area = cv2.contourArea(contour)
        if area > MIN_OBJECT_AREA:
            x, y, w, h = cv2.boundingRect(contour)  # Get bounding box coordinates

            if w == 0: 
                continue

            distance = estimate_distance(w)  # Use 'w' for distance

            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate safe zone 
            obj_safe_dis, obj_sa_dir = get_safe_zone_guidance(center_x)

            obj_data = {
                "X": center_x,
                "Y": center_y,
                "Dis": distance,
                "SafeDis": obj_safe_dis,
                "SaDir": obj_sa_dir,  
            }
            red_objects_details_unsorted.append(obj_data)
            red_distances_list.append(distance)  # For calculating overall average

            # Draw box and text on the detection frame
            cv2.rectangle(
                detection_frame, (x, y), (x + w, y + h), (0, 0, 255), 2
            )  # Red box
            cv2.putText(
                detection_frame,
                f"Red {distance:.2f}cm",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
            cv2.putText(
                detection_frame,
                f"ID {contour_idx+1} ({center_x},{center_y}) [{obj_sa_dir}]",  # Show direction
                (x, y + h + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )

    # --- Sort Red objects by distance  ---
    red_objects_details_unsorted.sort(key=lambda obj: obj["Dis"])

    #  JSON for Red objects
    if red_objects_details_unsorted:
        json_output["RedObject"]["IsRed"] = True
        json_output["RedObject"]["RedCounter"] = len(red_objects_details_unsorted)
        json_output["RedObject"]["RedDis"] = round(
            sum(red_distances_list) / len(red_distances_list), 2
        )

        for i, obj_data in enumerate(red_objects_details_unsorted):
            json_output["RedObject"]["RedAxis"][str(i + 1)] = obj_data

    # Detect Blue objects
    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    contours_blue, _ = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for contour_idx, contour in enumerate(contours_blue):
        area = cv2.contourArea(contour)
        if area > MIN_OBJECT_AREA:
            x, y, w, h = cv2.boundingRect(contour)  # Get bounding box coordinates

            if w == 0:
                continue

            distance = estimate_distance(w)  # Use 'w' for distance

            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate safe zone
            obj_safe_dis, obj_sa_dir = get_safe_zone_guidance(center_x)

            obj_data = {
                "X": center_x,
                "Y": center_y,
                "Dis": distance,
                "SafeDis": obj_safe_dis,  
                "SaDir": obj_sa_dir,  
            }
            blue_objects_details_unsorted.append(obj_data)
            blue_distances_list.append(distance)  

            # Draw box and text on the detection frame
            cv2.rectangle(
                detection_frame, (x, y), (x + w, y + h), (255, 0, 0), 2
            )  # Blue box
            cv2.putText(
                detection_frame,
                f"Blue {distance:.2f}cm",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )
            cv2.putText(
                detection_frame,
                f"ID {contour_idx+1} ({center_x},{center_y}) [{obj_sa_dir}]",  # Show direction
                (x, y + h + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                1,
            )

    # --- Sort Blue objects by distance ---
    blue_objects_details_unsorted.sort(key=lambda obj: obj["Dis"])

    # JSON for Blue objects
    if blue_objects_details_unsorted:
        json_output["BlueObject"]["IsBlue"] = True
        json_output["BlueObject"]["BlueCounter"] = len(blue_objects_details_unsorted)
        json_output["BlueObject"]["BlueDis"] = round(
            sum(blue_distances_list) / len(blue_distances_list), 2
        )

        for i, obj_data in enumerate(blue_objects_details_unsorted):
            json_output["BlueObject"]["BlueAxis"][str(i + 1)] = obj_data

    # --- Draw Safe Zone ---
    cv2.line(
        detection_frame,
        (SAFE_ZONE_LEFT_EDGE_X, 0),
        (SAFE_ZONE_LEFT_EDGE_X, FRAME_HEIGHT),
        SAFE_ZONE_LINE_COLOR,
        SAFE_ZONE_LINE_THICKNESS,
    )
    cv2.line(
        detection_frame,
        (SAFE_ZONE_RIGHT_EDGE_X, 0),
        (SAFE_ZONE_RIGHT_EDGE_X, FRAME_HEIGHT),
        SAFE_ZONE_LINE_COLOR,
        SAFE_ZONE_LINE_THICKNESS,
    )
    # Add text label for safe zone
    cv2.putText(
        detection_frame,
        "Safe Zone",
        (SAFE_ZONE_LEFT_EDGE_X + 5, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        SAFE_ZONE_LINE_COLOR,
        2,
    )
    # Draw safe zone center 
    cv2.circle(
        detection_frame, (SAFE_ZONE_CENTER_X, FRAME_HEIGHT // 2), 5, (255, 0, 255), -1
    ) 
    cv2.putText(
        detection_frame,
        "SZ Cntr",
        (SAFE_ZONE_CENTER_X + 10, FRAME_HEIGHT // 2 + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 0, 255),
        1,
    )

    # --- Print JSON to terminal ---
    print(json.dumps(json_output, indent=4))

    # --- Display Frames ---
    cv2.imshow("Original Frame", frame)
    cv2.imshow("General Mask", mask_general)
    cv2.imshow("General Result", result_general)
    cv2.imshow(
        "Red/Blue Detection", detection_frame
    )  # Display the frame with red/blue boxes and safe zone

    # --- Frame Rate Control and GUI Event Processing ---
    processing_time_ms = (time.time() - current_loop_start_time) * 1000
    wait_time_ms = max(
        MIN_WAIT_TIME_MS, int(frame_delay_target_ms - processing_time_ms)
    )
    key = cv2.waitKey(wait_time_ms) & 0xFF

    if key == ord("q"):
        print("Exiting program.")
        break

# --- Closing ---
cap.release()
cv2.destroyAllWindows()
