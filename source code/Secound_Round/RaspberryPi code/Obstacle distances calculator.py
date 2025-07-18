import numpy as np
import cv2
import serial
import time
import math
import utilis #This python script contains function to stack all the frames being analyzed

# --- Configuration ---
SERIAL_READY = 0 #Whether a serial device is connected or not
CAMERA_INDEX = 1    # Select which cam will be used  #1 - laptop's camera #0 - micropack webcam
MACHINE = 0  # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
COM_PORT = 3
TUNE_HSV = 1 # whether we want to tune the hsv color values for different image elements. 
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
    ser = serial.Serial(f'COM{COM_PORT}', 115200, timeout = 1.0)

def estimate_distance(perceived_dimension_px):
    """Estimates distance to an object given its perceived dimension in pixels.
    Uses KNOWN_WIDTH_CM and FOCAL_LENGTH_PX from global config."""
    if perceived_dimension_px == 0:
        return 0.0
    return round((KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / perceived_dimension_px, 2)

    # Define bounds for the general trackbar mask
lower_bound_green = np.array([25, 135, 50])
upper_bound_green = np.array([55, 255, 255])

lower_bound1_red = np.array([0, 181, 70])
upper_bound1_red = np.array([5, 255, 255])

lower_bound2_red = np.array([175, 181, 70])
upper_bound2_red = np.array([179, 255, 255])
    

if TUNE_HSV==1:
    utilis.createTrackbarWindow()
# # --- Initialize camera ---
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if SERIAL_READY: 
    ser.reset_input_buffer()
    time.sleep(1)
    ser.write('r'.encode('utf-8')) # indicates that the computer is ready for image processing
print("Serial: r")

serialFlag = 0
serialFlag2 = 0
# --- Video Processing Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame. Exiting...")
        break
    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Get Trackbar Positions for general HSV tuning ---
    if TUNE_HSV == 1:
        g_l_h = cv2.getTrackbarPos("Green L - H", "HSV Trackbars") #g_l_h = green object's lower hue value
        g_l_s = cv2.getTrackbarPos("Green L - S", "HSV Trackbars")
        g_l_v = cv2.getTrackbarPos("Green L - V", "HSV Trackbars")
        g_u_h = cv2.getTrackbarPos("Green U - H", "HSV Trackbars")
        g_u_s = cv2.getTrackbarPos("Green U - S", "HSV Trackbars")
        g_u_v = cv2.getTrackbarPos("Green U - V", "HSV Trackbars")

        r_l_h1 = cv2.getTrackbarPos("Red L - H1", "HSV Trackbars") #r_l_h = red object's lower hue value
        r_l_h2 = cv2.getTrackbarPos("Red L - H2", "HSV Trackbars") #r_l_h = red object's lower hue value

        r_l_s = cv2.getTrackbarPos("Red L - S", "HSV Trackbars")
        r_l_v = cv2.getTrackbarPos("Red L - V", "HSV Trackbars")

        r_u_h1 = cv2.getTrackbarPos("Red U - H1", "HSV Trackbars")
        r_u_h2 = cv2.getTrackbarPos("Red U - H2", "HSV Trackbars")

        r_u_s = cv2.getTrackbarPos("Red U - S", "HSV Trackbars")
        r_u_v = cv2.getTrackbarPos("Red U - V", "HSV Trackbars")

            # Define bounds for the general trackbar mask
        lower_bound_green = np.array([g_l_h, g_l_s, g_l_v])
        upper_bound_green = np.array([g_u_h, g_u_s, g_u_v])
        #blue_mask = cv2.inRange(hsv, lower_bound_green, upper_bound_green) #the output image has white pixels only where there's blue (according to the trackbar hsv range), all other pixels are black
        
        lower_bound1_red = np.array([r_l_h1, r_l_s, r_l_v])
        upper_bound1_red = np.array([r_u_h1, r_u_s, r_u_v])

        lower_bound2_red = np.array([r_l_h2, r_l_s, r_l_v])
        upper_bound2_red = np.array([r_u_h2, r_u_s, r_u_v])

    # Green object mask
    green_mask = cv2.inRange(hsv, lower_bound_green, upper_bound_green)
    green_masked_frame = cv2.bitwise_and(frame,frame,  mask = green_mask)
    #Why frame is passed two times in the above function?  Because cv2.bitwise_and() is designed to combine two images (pixel by pixel using the AND operation). If you want to apply a mask on a single image (frame), you simply bitwise AND it with itself. That way: Each pixel P in the result becomes: P = frame AND frame → which is just P, but only where the mask is non-zero.

       # 1. Detect Red objects
    red_mask1 = cv2.inRange(hsv, lower_bound1_red, upper_bound1_red)
    red_mask2 = cv2.inRange(hsv, lower_bound2_red, upper_bound2_red)
    red_mask_combined = cv2.bitwise_or(red_mask1, red_mask2)
    red_masked_frame = cv2.bitwise_and(frame, frame, mask = red_mask_combined)
    
    #blue_image = cv2.bitwise_and(frame, frame, mask = blue_mask) # In blue_image we only see the pixels that are white in blue_mask, so we only see the particular color of the hsv range, all other pixels are made black. 
    contours_red, _ = cv2.findContours(
        red_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
    )

    obstaclePresent = 0
    obstacleArea = 0
    for contour_id, contour in enumerate(contours_red):
        area = cv2.contourArea(contour)
        #print(area)
        x,y,w,h = cv2.boundingRect(contour) #Assuming that the camera's lense surface is parallel to one of the side of the wro obstacle
        if area > MIN_OBJECT_AREA and  w!=0: #If the distance reading has been reported once, we won't send it over and over again 
            obstaclePresent = 1
            obstacleArea = math.floor(area)
            break

    if obstaclePresent :
            if serialFlag==1:
                #distance = math.floor(estimate_distance(w))
                if SERIAL_READY==1: 
                    ser.write(f"R:{area};".encode('utf-8'))
                print(f"Serial: R:{area};")
                serialFlag = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
                #serialFlag2=1  # Marking that we have sent the distance once, and won't sent this over and over again
    elif serialFlag ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
                    if SERIAL_READY==1: 
                        ser.write("R:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
                    print("Serial: R:0; ")
                    serialFlag = 1 #We won't send this "No blue object in vision range" continuosly, we'll just send it once

    contours_green, _ = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
    )

    obstaclePresent = 0
    obstacleArea = 0
    for contour_id, contour in enumerate(contours_green):
        area = cv2.contourArea(contour)
        #print(area)
        x,y,w,h = cv2.boundingRect(contour) #Assuming that the camera's lense surface is parallel to one of the side of the wro obstacle
        if area > MIN_OBJECT_AREA and  w!=0: #If the distance reading has been reported once, we won't send it over and over again 
            obstaclePresent = 1
            obstacleArea = math.floor(area)
            break

    if obstaclePresent :
            if serialFlag2==1:
                #distance = math.floor(estimate_distance(w))
                if SERIAL_READY==1: 
                    ser.write(f"G:{area};".encode('utf-8'))
                print(f"Serial: G:{area};")
                serialFlag2 = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
                #serialFlag2=1  # Marking that we have sent the distance once, and won't sent this over and over again
    elif serialFlag2 ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
                    if SERIAL_READY==1: 
                        ser.write("G:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
                    print("Serial: G:0; ")
                    serialFlag2 = 1
     
    stackedImages = utilis.stackImages(0.6, ([frame, green_masked_frame, red_masked_frame], [frame, frame, frame]))
    cv2.imshow("Frames", stackedImages)
    key = cv2.waitKey(1)  #Purpose of the above expression: It waits for a speciefied amount of time(in milliseconds) for a key event to occur. This small delay is crucial when processing video streams, as it allows the system to display each frame for a brief period, creating the illusion of continuosu motion. Without this delay, the frames would be processed and displayed so quickly that the video would appear as a blur or not be visible at all. And in most cases, the window will have "Not responding" problem and ultimately crash. During this delay, cv2.waitKey(1) also checks if any key has been pressed. If any key has been prssed. If a key is pressed within the 1-millisecond window, it returns the ascii value fo that pressed key. If no key is pressed within that time, it returns -1. 
    if key == ord('q'): # Stops the execution of the entire python program
        if SERIAL_READY==1:
            ser.close()
        break
    elif key == ord('s'):  #Stops/Starts the car
         if SERIAL_READY:
              ser.write("x;".encode('utf-8')) 
         print("Serial: x;")
         


