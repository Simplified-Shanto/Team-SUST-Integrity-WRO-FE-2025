#-----Full forms------------------
#LLMC - Low level microcontroller
#SBC - Single Board Computer
# --- Configuration ---
#!/usr/bin/env python3
DEVELOPING   = 0 # The code is in development mode, and we'll show processed images at different stages, 
                 # otherwise, there'll be no ui output of the code thus we can run it headless on startup i
                 # in raspberry pie. 
FOCAL_LENGTH_PX = 535 #Focal length in pixels - 530 for micropack webcam
SERIAL_READY = 1 #Whether a serial device is connected or not
CAMERA_INDEX = 0    # Select which cam will be used  #1 - laptop's camera #0 - micropack webcam
MACHINE = 0  # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
COM_PORT = 4
TUNE_HSV = 0 # whether we want to tune the hsv color values for different image elements. 

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MIN_OBJECT_AREA = 500  # Minimum contour area to consider an object (adjust as needed)
MIN_LINE_AREA = 500

import numpy as np
import cv2
import serial
import time
import math

KNOWN_WIDTH_CM = 5.0  ## Here we are using teh WRO future engineers game obstacle as sample object Object's physical width # KNOWN_WIDTH_CM should correspond to the 'w' (width)  of the object
KNOWN_DISTANCE_CM = 30

if SERIAL_READY==1 and MACHINE == 1:
    ser = serial.Serial("/dev/esp32_serial", 115200, timeout = 1)
    time.sleep(2)
    # At startup we have a fresh buffer with nothing in it. 
    ser.reset_input_buffer()
elif SERIAL_READY==1 and MACHINE ==0: 
    ser = serial.Serial(f'COM{COM_PORT}', 115200, timeout = 1.0)
    time.sleep(2)
    ser.reset_input_buffer()


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver
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

blue_line_lower_bound = np.array([99, 40 , 90 ])
blue_line_upper_bound = np.array([135, 255, 255 ])

orange_line_lower_bound = np.array([10, 150, 150 ])
orange_line_upper_bound = np.array([24, 255, 255 ])



if TUNE_HSV==1 and DEVELOPING==1: 
    cv2.namedWindow("Vehicle Parameters", cv2.WINDOW_NORMAL)  # This window will be used to tune different parameters of the vehicle, like speed, pid values etc via serial commands
    cv2.resizeWindow("Vehicle Parameters", 600, 400)
    cv2.waitKey(100)

    def changeVehicleSpeed(speed):
        if SERIAL_READY: 
            ser.write(f"s:{speed};".encode("utf-8"))
        if DEVELOPING==1: 
            print(f"Serial: s:{speed};")
     
    cv2.createTrackbar("Speed", "Vehicle Parameters", 100, 255, changeVehicleSpeed)
    def nothing(x):
        pass
     # --- Create General Trackbar Window
    cv2.namedWindow("HSV Trackbars", cv2.WINDOW_NORMAL) #WINDOW_NORMAL is a flag that let's user resize the window, by default its, cv2.AUTOSIZE, which makes the window fit to the size of the content and it is unresizable. 
    cv2.resizeWindow("HSV Trackbars", 600, 600)
    cv2.waitKey(100)  # Wait 100ms for the window to draw
    # Create trackbars for general HSV lower and upper bounds
    #                                          I            ntial value of trackbar    Highest value of the trackbar
    cv2.createTrackbar("Green L - H", "HSV Trackbars", lower_bound_green[0],                    179,             nothing)
    cv2.createTrackbar("Green L - S", "HSV Trackbars", lower_bound_green[1], 255, nothing)
    cv2.createTrackbar("Green L - V", "HSV Trackbars", lower_bound_green[2], 255, nothing)
    cv2.createTrackbar("Green U - H", "HSV Trackbars", upper_bound_green[0], 179, nothing)
    cv2.createTrackbar("Green U - S", "HSV Trackbars", upper_bound_green[1], 255, nothing)
    cv2.createTrackbar("Green U - V", "HSV Trackbars", upper_bound_green[2], 255, nothing)
    # HSV tuning trackbars for red objects
    cv2.createTrackbar("Red L - H1", "HSV Trackbars", lower_bound1_red[0], 179, nothing)
    cv2.createTrackbar("Red L - H2", "HSV Trackbars", lower_bound2_red[0], 179, nothing)
    cv2.createTrackbar("Red L - S", "HSV Trackbars", lower_bound1_red[1], 255, nothing)
    cv2.createTrackbar("Red L - V", "HSV Trackbars", lower_bound1_red[2], 255, nothing)
    cv2.createTrackbar("Red U - H1", "HSV Trackbars", upper_bound1_red[0], 179, nothing)
    cv2.createTrackbar("Red U - H2", "HSV Trackbars", upper_bound2_red[0], 179, nothing)
    cv2.createTrackbar("Red U - S", "HSV Trackbars", upper_bound2_red[1], 255, nothing)
    cv2.createTrackbar("Red U - V", "HSV Trackbars", upper_bound2_red[2], 255, nothing)

    cv2.namedWindow("Line HSV trackbars", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Line HSV trackbars", 600, 600)
    cv2.waitKey(100) 
    cv2.createTrackbar("Blue Line L_H", "Line HSV trackbars",blue_line_lower_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line L_S", "Line HSV trackbars",blue_line_lower_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line L_V", "Line HSV trackbars",blue_line_lower_bound[2], 255, nothing )
    cv2.createTrackbar("Blue Line U_H", "Line HSV trackbars",blue_line_upper_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line U_S", "Line HSV trackbars",blue_line_upper_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line U_V", "Line HSV trackbars",blue_line_upper_bound[2], 255, nothing )

    cv2.createTrackbar("Orange Line L_H", "Line HSV trackbars", orange_line_lower_bound[0], 179, nothing)
    cv2.createTrackbar("Orange Line L_S", "Line HSV trackbars", orange_line_lower_bound[1], 179, nothing)
    cv2.createTrackbar("Orange Line L_V", "Line HSV trackbars", orange_line_lower_bound[2], 179, nothing)
    cv2.createTrackbar("Orange Line U_H", "Line HSV trackbars", orange_line_upper_bound[0], 179, nothing)
    cv2.createTrackbar("Orange Line U_S", "Line HSV trackbars", orange_line_upper_bound[1], 179, nothing)
    cv2.createTrackbar("Orange Line U_V", "Line HSV trackbars", orange_line_upper_bound[2], 179, nothing)

# # --- Initialize camera ---
if MACHINE == 0:  # Windows
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
else:             # Linux - Raspberry Pi
    cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if SERIAL_READY: 
    ser.reset_input_buffer()
    time.sleep(1)
    ser.write('r'.encode('utf-8')) # indicates that the SBC is ready for image processing
    if DEVELOPING:
        print("Serial: r")

#-------Flags------------------
serialFlag = 0      # Whether we've sent the distance of the red obstacle to the LLM 
serialFlag2 = 0     # Whether we've sent the distance of the green obstacle to the LLM
directionSentFlag = 0 # Whether we've reported the direction of the round to the LLM

# --- Video Processing Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        if DEVELOPING:
            print("Error: Failed to grab frame. Exiting...")
        break
    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cropped_frame = frame[100:400, 150:590]
    cropped_hsv   = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
    blue_line_mask = cv2.inRange(cropped_hsv, blue_line_lower_bound, blue_line_upper_bound)
    blue_line_masked_frame = cv2.bitwise_and(cropped_frame, cropped_frame, mask = blue_line_mask)
    
    orange_line_mask = cv2.inRange(cropped_hsv, orange_line_lower_bound, orange_line_upper_bound)
    orange_line_masked_frame = cv2.bitwise_and(cropped_frame, cropped_frame, mask = orange_line_mask)
    combined_line_mask = cv2.bitwise_or(blue_line_masked_frame, orange_line_masked_frame)

    blue_line_contours, _ = cv2.findContours(blue_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    orange_line_contours, _ = cv2.findContours(orange_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if directionSentFlag == 0: #If we haven't send the round direction to the LLM yet
        #This basically measures, which color of threshold area do we get first
        #Checking for blue line
        for contour_index, contour in enumerate(blue_line_contours): 
            area = cv2.contourArea(contour)
            #print("blue = ", area)
            if cv2.contourArea(contour) > MIN_LINE_AREA:
                if SERIAL_READY:
                    ser.write("b;".encode('utf-8'))
                if DEVELOPING:
                    print("Serial: b;")
                directionSentFlag = 1
                break
        #Checking for orange line
        for cntour_index, contour in enumerate(orange_line_contours): 
            area = cv2.contourArea(contour)
            #print("orange = ", area)
            if area > MIN_LINE_AREA:
                if SERIAL_READY:
                    ser.write("o;".encode('utf-8'))
                if DEVELOPING: 
                    print("Serial: o;")
                directionSentFlag = 1
                break
            
    # --- Get Trackbar Positions for general HSV tuning ---
    if TUNE_HSV == 1 and DEVELOPING==1:
        g_l_h = cv2.getTrackbarPos("Green L - H", "HSV Trackbars") #g_l_h = green object's lower hue value
        g_l_s = cv2.getTrackbarPos("Green L - S", "HSV Trackbars")
        g_l_v = cv2.getTrackbarPos("Green L - V", "HSV Trackbars")
        g_u_h = cv2.getTrackbarPos("Green U - H", "HSV Trackbars")
        g_u_s = cv2.getTrackbarPos("Green U - S", "HSV Trackbars")
        g_u_v = cv2.getTrackbarPos("Green U - V", "HSV Trackbars")
                    # Define bounds for the general trackbar mask
        lower_bound_green = np.array([g_l_h, g_l_s, g_l_v])
        upper_bound_green = np.array([g_u_h, g_u_s, g_u_v])

        r_l_h1 = cv2.getTrackbarPos("Red L - H1", "HSV Trackbars") #r_l_h = red object's lower hue value
        r_l_h2 = cv2.getTrackbarPos("Red L - H2", "HSV Trackbars") #r_l_h = red object's lower hue value

        r_l_s = cv2.getTrackbarPos("Red L - S", "HSV Trackbars")
        r_l_v = cv2.getTrackbarPos("Red L - V", "HSV Trackbars")

        r_u_h1 = cv2.getTrackbarPos("Red U - H1", "HSV Trackbars")
        r_u_h2 = cv2.getTrackbarPos("Red U - H2", "HSV Trackbars")

        r_u_s = cv2.getTrackbarPos("Red U - S", "HSV Trackbars")
        r_u_v = cv2.getTrackbarPos("Red U - V", "HSV Trackbars")

        lower_bound1_red = np.array([r_l_h1, r_l_s, r_l_v])
        upper_bound1_red = np.array([r_u_h1, r_u_s, r_u_v])

        lower_bound2_red = np.array([r_l_h2, r_l_s, r_l_v])
        upper_bound2_red = np.array([r_u_h2, r_u_s, r_u_v])
    # cv2.createTrackbar("Blue Line L_V", "Line HSV trackbars",255, 255, nothing )

    # cv2.createTrackbar("Orange Line L_H", "Line HSV trackbars", 0, 179, nothing)
        bl_l_h = cv2.getTrackbarPos("Blue Line L_H", "Line HSV trackbars") #bl_l_h = blue line lower hue
        bl_l_s = cv2.getTrackbarPos("Blue Line L_S", "Line HSV trackbars")
        bl_l_v = cv2.getTrackbarPos("Blue Line L_V", "Line HSV trackbars")
    
        bl_u_h = cv2.getTrackbarPos("Blue Line U_H", "Line HSV trackbars") 
        bl_u_s = cv2.getTrackbarPos("Blue Line U_S", "Line HSV trackbars")
        bl_u_v = cv2.getTrackbarPos("Blue Line U_V", "Line HSV trackbars")
        blue_line_lower_bound = np.array([bl_l_h, bl_l_s , bl_l_v ])
        blue_line_upper_bound = np.array([bl_u_h, bl_u_s,  bl_u_v ])

        or_l_h = cv2.getTrackbarPos("Orange Line L_H", "Line HSV trackbars") #or_l_h = orange line lower hue
        or_l_s = cv2.getTrackbarPos("Orange Line L_S", "Line HSV trackbars")
        or_l_v = cv2.getTrackbarPos("Orange Line L_V", "Line HSV trackbars")

        or_u_h = cv2.getTrackbarPos("Orange Line U_H", "Line HSV trackbars") #or_l_h = orange line lower hue
        or_u_s = cv2.getTrackbarPos("Orange Line U_S", "Line HSV trackbars")
        or_u_v = cv2.getTrackbarPos("Orange Line U_V", "Line HSV trackbars")
    

    # Green object mask
    green_mask = cv2.inRange(hsv, lower_bound_green, upper_bound_green)
    green_masked_frame = cv2.bitwise_and(frame,frame,  mask = green_mask)
    #Why frame is passed two times in the above function?  Because cv2.bitwise_and() is designed to combine two images (pixel by pixel using the AND operation). If you want to apply a mask on a single image (frame), you simply bitwise AND it with itself. That way: Each pixel P in the result becomes: P = frame AND frame → which is just P, but only where the mask is non-zero.
    contours_green, _ = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
    )
    obstaclePresent = 0
    obstacleArea = 0
    for contour_id, contour in enumerate(contours_green):
        area = cv2.contourArea(contour)
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
                if DEVELOPING:
                    print(f"Serial: G:{area};")
                serialFlag2 = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
                #serialFlag2=1  # Marking that we have sent the distance once, and won't sent this over and over again
    elif serialFlag2 ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
                    if SERIAL_READY==1: 
                        ser.write("G:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
                    if DEVELOPING:    
                        print("Serial: G:0; ")
                    serialFlag2 = 1


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
                if DEVELOPING:
                    print(f"Serial: R:{area};")
                serialFlag = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
                #serialFlag2=1  # Marking that we have sent the distance once, and won't sent this over and over again
    elif serialFlag ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
                    if SERIAL_READY==1: 
                        ser.write("R:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
                    if DEVELOPING:
                        print("Serial: R:0; ")
                    serialFlag = 1 #We won't send this "No blue object in vision range" continuosly, we'll just send it once

  
    stackedImages = stackImages(0.6, ([frame, green_masked_frame, red_masked_frame],
                                             [blue_line_masked_frame, orange_line_masked_frame, combined_line_mask]))
   
    if DEVELOPING:
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
        elif key == ord('r'):
            directionSentFlag = 0
         
    



