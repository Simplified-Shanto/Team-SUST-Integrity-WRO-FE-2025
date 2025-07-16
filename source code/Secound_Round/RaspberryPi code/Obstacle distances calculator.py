import numpy as np
import cv2
import serial
import time
import math

# --- Configuration ---
MACHINE = 0 # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
SERIAL_READY = 1 #Whether a serial device is connected or not
CAMERA_INDEX = 1  # Select which cam will be used
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MIN_OBJECT_AREA = 500  # Minimum contour area to consider an object (adjust as needed)
# Object dimensions and camera focal length for distance estimation (in cm and px)
# KNOWN_WIDTH_CM should correspond to the 'w' (width)  of the object
# Here we are using teh WRO future engineers game obstacle as sample object
KNOWN_WIDTH_CM = 5.0  # Object's physical width
KNOWN_DISTANCE_CM = 30
FOCAL_LENGTH_PX = 535 #Focal length in pixels - 530 for micropack webcam

if SERIAL_READY and MACHINE == 1:
    ser = serial.Serial('/dev/ttyACM2', 115200, timeout = 1.0)
elif SERIAL_READY and MACHINE == 0:
    ser = serial.Serial('COM17', 115200, timeout = 1.0)

def estimate_distance(perceived_dimension_px):
    """Estimates distance to an object given its perceived dimension in pixels.
    Uses KNOWN_WIDTH_CM and FOCAL_LENGTH_PX from global config."""
    if perceived_dimension_px == 0:
        return 0.0
    return round((KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / perceived_dimension_px, 2)

def nothing(x):
    pass
# --- Create General Trackbar Window
cv2.namedWindow("HSV Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Trackbars", 600, 300)
cv2.waitKey(100)  # Wait 100ms for the window to draw
# Create trackbars for general HSV lower and upper bounds
#                                          Intial value of trackbar    Highest value of the trackbar
cv2.createTrackbar("L - H", "HSV Trackbars",          100,                           179,             nothing)
cv2.createTrackbar("L - S", "HSV Trackbars", 150, 255, nothing)
cv2.createTrackbar("L - V", "HSV Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - H", "HSV Trackbars", 140, 179, nothing)
cv2.createTrackbar("U - S", "HSV Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "HSV Trackbars", 255, 255, nothing)

# --- Initialize camera ---
cap = cv2.VideoCapture(CAMERA_INDEX)
 # Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if SERIAL_READY: 
    ser.reset_input_buffer()
    time.sleep(1)
    ser.write('r'.encode('utf-8')) # indicates that the computer is ready for image processing
else: 
    print("Serial: r")

serialFlag = 0
# --- Video Processing Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame. Exiting...")
        break
    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
   
    # detection_frame = frame.copy()
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

    blue_mask = cv2.inRange(hsv, lower_bound_blue, upper_bound_blue) #the output image has white pixels only where there's blue (according to the trackbar hsv range), all other pixels are black
    #blue_image = cv2.bitwise_and(frame, frame, mask = blue_mask) # In blue_image we only see the pixels that are white in blue_mask, so we only see the particular color of the hsv range, all other pixels are made black. 
    contours_blue, _ = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
    )

    for contour_id, contour in enumerate(contours_blue):
        area = cv2.contourArea(contour)
        if area > MIN_OBJECT_AREA: 
            x,y,w,h = cv2.boundingRect(contour) #Assuming that the camera's lense surface is parallel to one of the side of the wro obstacle
            if w==0: 
                continue
            distance = math.floor(estimate_distance(w))
            if SERIAL_READY==1: 
                ser.write(f"G:{distance};".encode('utf-8'))
            else: 
                print(f"Serial: G:{distance};")
            serialFlag = 0
        elif serialFlag ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
                    if SERIAL_READY==1: 
                        ser.write("G:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
                    else: 
                        print("Serial: G:0; ")
                    serialFlag = 1 #We won't send this "No blue object in vision range" continuosly, we'll just send it once



            
                

    cv2.imshow("blue_mask", blue_mask)
    cv2.imshow("RawFrame", frame)
    key = cv2.waitKey(10)  #Purpose of the above expression: It waits for a speciefied amount of time(in milliseconds) for a key event to occur. This small delay is crucial when processing video streams, as it allows the system to display each frame for a brief period, creating the illusion of continuosu motion. Without this delay, the frames would be processed and displayed so quickly that the video would appear as a blur or not be visible at all. And in most cases, the window will have "Not responding" problem and ultimately crash. During this delay, cv2.waitKey(1) also checks if any key has been pressed. If any key has been prssed. If a key is pressed within the 1-millisecond window, it returns the ascii value fo that pressed key. If no key is pressed within that time, it returns -1. 
    if key == ord('q'): 
        ser.close()
        break


