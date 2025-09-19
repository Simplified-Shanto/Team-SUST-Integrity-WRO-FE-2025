#-----Full forms------------------
#LLMC - Low level microcontroller
#SBC - Single Board Computer

#!/usr/bin/env python3

# --- Configuration ---
MACHINE = 1  # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
DEVELOPING   = 1 # The code is in development mode, and we'll show processed images at different stages, 
                 # otherwise, there'll be no ui output of the code thus we can run it headless on startup i
                 # in raspberry pie. 
 
SERIAL_READY = 1 #Whether a serial device is connected or not
CAM_TYPE = 0 # 0  = Raspicamera, 1  = webcam.
FOCAL_LENGTH_PX = 335 #Focal length in pixels - 530 for micropack webcam 335 - raspi cam
BRIGHT_LIGHT = 0 # Bright_light = 1 indicates that we are testing things in bright daylight, bright_light = 0 means that we are testing this thing in night under led lights
TUNE_HSV = 0 # whether we want to tune the hsv color values for different image elements. 

CAMERA_INDEX = 0    # Select which cam will be used  #1 - laptop's camera #0 - micropack webcam
COM_PORT = 4
TUNE_VEHICLE_PARAMETERS = 0
SHOW_LINE_ANALYSIS = 0

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH/2

MIN_OBJECT_AREA = 500  # Minimum contour area to consider an object (adjust as needed)
MIN_LINE_AREA = 1000
frontDistance = 35

if MACHINE == 1 and CAM_TYPE==0: 
    from picamera2 import Picamera2
import numpy as np
import cv2
import serial
import time
import math
import os

KNOWN_HEIGHT_CM = 10.0  ## Here we are using teh WRO future engineers game obstacle as sample object Object's physical width # KNOWN_WIDTH_CM should correspond to the 'w' (width)  of the object
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
def estimate_distance(perceived_dimension_px): # Input the height of the bounding box or the rectangular contour detected. 
    """Estimates distance to an object given its perceived dimension in pixels.
    Uses KNOWN_WIDTH_CM and FOCAL_LENGTH_PX from global config."""
    if perceived_dimension_px == 0:
        return 0.0
    return round((KNOWN_HEIGHT_CM * FOCAL_LENGTH_PX) / perceived_dimension_px, 2)
    #return round((KNOWN_DISTANCE_CM * perceived_dimension_px) / KNOWN_HEIGHT_CM) # Uncomment this one to find the focal length of the camera by placing the object at the known distance
    
    

if CAM_TYPE==1: # Webcam
    #Define bounds for the general trackbar mask (works fine in bright light (day) condition with webcam ) 
    lower_bound_green = np.array([15, 60, 0])
    upper_bound_green = np.array([46, 255, 255])

    lower_bound1_red = np.array([170, 70, 0])
    upper_bound1_red = np.array([179, 255, 255])

    lower_bound2_red = np.array([0, 70, 00])
    upper_bound2_red = np.array([5, 255, 255])

    blue_line_lower_bound = np.array([101, 120 , 00 ])
    blue_line_upper_bound = np.array([167, 255, 255 ])

    orange_line_lower_bound = np.array([6, 60, 0 ])
    orange_line_upper_bound = np.array([20, 255, 255 ])

elif CAM_TYPE==0:
    if BRIGHT_LIGHT==1:    # Define bounds for the general trackbar mask (works fine in bright light condition with raspi cam ) 
        lower_bound_green = np.array([15, 10, 0])
        upper_bound_green = np.array([46, 255, 255])
        1
        lower_bound1_red = np.array([165, 190, 0])
        upper_bound1_red = np.array([179, 255, 255])

        lower_bound2_red = np.array([165, 190, 00])
        upper_bound2_red = np.array([179, 255, 255])

        blue_line_lower_bound = np.array([111, 93 , 00 ])
        blue_line_upper_bound = np.array([150, 255, 255 ])

        orange_line_lower_bound = np.array([174, 102, 14 ])
        orange_line_upper_bound = np.array([179, 170, 255 ])
    elif BRIGHT_LIGHT==0:
            ##Define bounds for the general trackbar mask (works fine in low light condition with Raspi cam) 
        lower_bound_green = np.array([25, 135, 50])
        upper_bound_green = np.array([55, 255, 255])

        lower_bound1_red = np.array([0, 181, 70])
        upper_bound1_red = np.array([5, 255, 255])

        lower_bound2_red = np.array([175, 181, 70])
        upper_bound2_red = np.array([179, 255, 255])

        blue_line_lower_bound = np.array([87, 48 , 0 ])
        blue_line_upper_bound = np.array([144, 255, 255 ])

        orange_line_lower_bound = np.array([6, 85, 0])
        orange_line_upper_bound = np.array([25, 255, 255 ])


if TUNE_HSV==1 and DEVELOPING==1: 
    if TUNE_VEHICLE_PARAMETERS==1:
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
    cv2.createTrackbar("Orange Line L_S", "Line HSV trackbars", orange_line_lower_bound[1], 255, nothing)
    cv2.createTrackbar("Orange Line L_V", "Line HSV trackbars", orange_line_lower_bound[2], 255, nothing)
    cv2.createTrackbar("Orange Line U_H", "Line HSV trackbars", orange_line_upper_bound[0], 179, nothing)
    cv2.createTrackbar("Orange Line U_S", "Line HSV trackbars", orange_line_upper_bound[1], 255, nothing)
    cv2.createTrackbar("Orange Line U_V", "Line HSV trackbars", orange_line_upper_bound[2], 255, nothing)

# # --- Initialize camera ---
if MACHINE == 0:  # Windows
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

elif MACHINE==1:             # Linux / Raspberry Pi
    if CAM_TYPE==0: # Pi camera 
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH,FRAME_HEIGHT)})
        picam2.configure(config)
        picam2.start()
    elif CAM_TYPE==1:  # USB webcam. 
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
roundDirection = 0  # 1 = clockwise (orange line comes before blue) -1 = anti-clockwise (blue line comes before orange )

lineInterval = 1000  # the interval(ms) between counting lines of a particular color. 
stopDelay = 1000     # time (ms) to wait after counting the last line and stopping the vehicle. 

blue_line_count = -1  # -1 signifies that, line counting hasn't started yet. 
blue_line_timer = time.time() * 1000 # Getting the total execution time in millisecond
orange_line_timer = blue_line_timer
orange_line_count = -1

setPoint = 120 # We want the object to be in the center of the frame. 
carStopped = 0 # This scripts assumes that, the car is in motion in the beginning of this script. 


# --- Video Processing Loop ---
while True:
    
    if SERIAL_READY ==1 and ser.in_waiting > 0:  # If there's some message from Arduino
        command = ser.readline().decode('utf-8').strip()  # Read line & strip newline/spaces
        if DEVELOPING == 1:
            print("Raw command = ", command)
        # Case 1: simple one-letter command like 'r' or 'd'
        if command == "r":   # The lap is starting via button press, so start counting lines
            blue_line_count = 0
            orange_line_count = 0
            directionSentFlag = 0; 
            if DEVELOPING: print("Lap started (reset line count and round direction).")

        elif command == "d" : 
            if MACHINE==1: #Linux 
                os.system("sudo shutdown now") # Shutdown immediately
            elif MACHINE==0: # Windows
                if SERIAL_READY:
                    ser.close()
                break   #Stop execution of the script
        else:
            # Case 2: format 'char:value;' (e.g., 'p:1.23;')
            if ':' in command:
                try:
                    constant_name, value_str = command.split(":", 1)
                    constant_value = float(value_str)

                    if DEVELOPING:
                        print(f"constant_name = {constant_name}, constant_value = {constant_value}")

                    # Handle based on the constant_name
                    if constant_name == 'a':
                        lineInterval = constant_value
                    elif constant_name == 'b':
                        stopDelay = constant_value
                    else:
                        if DEVELOPING: print("Unknown constant:", constant_name)

                except ValueError:
                    if DEVELOPING: print("Invalid format received:", command)

            else:
                # If it's not in the expected format, treat it as your lineInterval
                lineInterval = command
                if DEVELOPING:
                    print("Line Interval = ", lineInterval)


    if DEVELOPING == 1 or blue_line_count!=-1: # do all the image processing, either if we are developing the code, or we are running a lap. 
        if CAM_TYPE==1:
            success, frame = cap.read()
        elif CAM_TYPE==0: 
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cropped_frame = frame[100:400, 150:590]
        cropped_hsv   = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

        blue_line_mask = cv2.inRange(cropped_hsv, blue_line_lower_bound, blue_line_upper_bound)
        blue_line_masked_frame = cv2.bitwise_and(cropped_frame, cropped_frame, mask = blue_line_mask)
        #cv2.putText(blue_line_masked_frame, f"Blue Lines: {blue_line_count}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 0, 0), 1)
        orange_line_mask = cv2.inRange(cropped_hsv, orange_line_lower_bound, orange_line_upper_bound)
        orange_line_masked_frame = cv2.bitwise_and(cropped_frame, cropped_frame, mask = orange_line_mask)
        #cv2.putText(orange_line_masked_frame, f"Orange Lines: {orange_line_count}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 1)
        blue_line_contours, _ = cv2.findContours(blue_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_line_contours, _ = cv2.findContours(orange_line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
         #If we haven't send the round direction to the LLM yet
            #This basically measures, which color of threshold area do we get first
            #Checking for blue line
        current_time = time.time()*1000
    
        # if blue_line_count!=-1:  # We'll only do the following processes when blue_line_count is set to zero by pressing the game start button in the vehicle and receiving serial command 'r' from the LLMC. The reason is avoiding early count of the lines by environmental noise before the round has started. 
        #     for contour_index, contour in enumerate(blue_line_contours): 
        #         area = cv2.contourArea(contour)
        #         #print("blue = ", area)
        #         if area > MIN_LINE_AREA:
        #             if(current_time - blue_line_timer > lineInterval):
        #                 blue_line_count +=1
        #                 blue_line_timer = current_time

        #             if directionSentFlag == 0:
        #                 if SERIAL_READY:
        #                     ser.write("b;".encode('utf-8'))
        #                 if DEVELOPING:
        #                     print("Serial: b;")
        #                 directionSentFlag = 1  # Round is anticlockwise
               
                        
            #Checking for orange line
            # for cntour_index, contour in enumerate(orange_line_contours): 
            #     area = cv2.contourArea(contour)
            #     #print("orange = ", area)
            #     if area > MIN_LINE_AREA:
            #         if(current_time - orange_line_timer > lineInterval):
            #             orange_line_count +=1
            #             orange_line_timer = current_time
            #         if directionSentFlag == 0:
            #             if SERIAL_READY:
            #                 ser.write("o;".encode('utf-8'))
            #             if DEVELOPING: 
            #                 print("Serial: o;")
            #             directionSentFlag = -1 # Round is clockwise
            
            # Checking for lap completion 
            # if orange_line_count==12:
            #     if DEVELOPING==1: 
            #         print("3 laps done. Waiting for RESET command"); 
            #     if SERIAL_READY==1: 
            #         message = 'x;' #Commands to stop the car. 
            #         time.sleep(stopDelay/1000)  # Waiting a bit to reach the center fo the tunnel. 
            #         ser.write(message.encode('utf-8'))
            #     #time.sleep(1)
            #     line_count = -1  # We won't count lines until a new lap is started by pressing the button

        

     
                
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

            ol_l_h = cv2.getTrackbarPos("Orange Line L_H", "Line HSV trackbars") #ol_l_h = orange line lower hue
            ol_l_s = cv2.getTrackbarPos("Orange Line L_S", "Line HSV trackbars")
            ol_l_v = cv2.getTrackbarPos("Orange Line L_V", "Line HSV trackbars")

            ol_u_h = cv2.getTrackbarPos("Orange Line U_H", "Line HSV trackbars") #ol_l_h = orange line lower hue
            ol_u_s = cv2.getTrackbarPos("Orange Line U_S", "Line HSV trackbars")
            ol_u_v = cv2.getTrackbarPos("Orange Line U_V", "Line HSV trackbars")

            orange_line_lower_bound = np.array([ol_l_h, ol_l_s , ol_l_v ])
            orange_line_upper_bound = np.array([ol_u_h, ol_u_s,  ol_u_v ])
        
        # Green object mask
        green_mask = cv2.inRange(hsv, lower_bound_green, upper_bound_green)
        green_masked_frame = cv2.bitwise_and(frame,frame,  mask = green_mask)
        if TUNE_HSV==1:
            cv2.putText(green_masked_frame, "Green object", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2); 
        #Why frame is passed two times in the above function?  Because cv2.bitwise_and() is designed to combine two images (pixel by pixel using the AND operation). If you want to apply a mask on a single image (frame), you simply bitwise AND it with itself. That way: Each pixel P in the result becomes: P = frame AND frame → which is just P, but only where the mask is non-zero.
        contours_green, _ = cv2.findContours(
            green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE   
        )
        obstaclePresent = 0
        obstacleArea = 0
        # for contour_id, contour in enumerate(contours_green):
        #     area = cv2.contourArea(contour)
        #     if area > MIN_OBJECT_AREA: #If the distance reading has been reported once, we won't send it over and over again 
        #         x,y,w,h = cv2.boundingRect(contour) #Assuming that the camera's lense surface is parallel to one of the side of the wro obstacle
        #         distance = math.floor(estimate_distance(h))
        #         if DEVELOPING:
        #             cv2.rectangle(green_masked_frame, (x, y), (x + w, y + h), (255, 255,0), 2)
        #             cv2.putText(green_masked_frame, str(distance), (int(x+(w/2) - 10), int(y - 20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2 ); 
        #         obstaclePresent = 1
        #         break

        # if obstaclePresent :
        #         if serialFlag2==1:
        #             if SERIAL_READY==1: 
        #                 ser.write(f"G:{distance};".encode('utf-8'))
        #             if DEVELOPING:
        #                 print(f"Serial: G:{distance};")
        #             serialFlag2 = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
        # elif serialFlag2 ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
        #                 if SERIAL_READY==1: 
        #                     ser.write("G:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
        #                 if DEVELOPING:    
        #                     print("Serial: G:0; ")
        #                 serialFlag2 = 1

        # 1. Detect Red objects
        red_mask1 = cv2.inRange(hsv, lower_bound1_red, upper_bound1_red)
        red_mask2 = cv2.inRange(hsv, lower_bound2_red, upper_bound2_red)
        red_mask_combined = cv2.bitwise_or(red_mask1, red_mask2)
        red_masked_frame = cv2.bitwise_and(frame, frame, mask = red_mask_combined)
        if TUNE_HSV==1:
            cv2.putText(red_masked_frame, "Red object", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2); 
        contours_red, _ = cv2.findContours(
            red_mask_combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obstaclePresent = 0
        obstacleArea = 0

        distance = 0
        for contour_id, contour in enumerate(contours_red):
            area = cv2.contourArea(contour)
            if area > MIN_OBJECT_AREA: #If the distance reading has been reported once, we won't send it over and over again 
                x,y,w,h = cv2.boundingRect(contour) #Assuming that the camera's lense surface is parallel to one of the side of the wro obstacle
                object_center_x = int(x + (w/2))
                value = int(FRAME_CENTER_X - object_center_x)
                error = value - setPoint
                distance = math.floor(estimate_distance(h))
                if DEVELOPING:
                    cv2.rectangle(red_masked_frame, (x, y), (x + w, y + h), (255, 255,0), 2)
                    cv2.putText(red_masked_frame, str(distance), (int(x+(w/2) - 10), int(y - 20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2 ); 
                obstaclePresent = 1
                break

        if obstaclePresent:
               if SERIAL_READY==1: 
                    if distance < frontDistance: 
                        ser.write(f"R:{distance}".encode('utf-8'))
                    elif distance > frontDistance: 
                        ser.write(f"E:{error};".encode('utf-8'))

                    if DEVELOPING:
                        print(f"Serial: R:{error};")
 
           

        #            serialFlag = 0  #The object is present in front of the vehicle, now we can send a 0 distance while it goes beyond the vision range. 
        # elif serialFlag ==0: # Camera is not encountering any blue object and we haven't reported it to the LLM 
        #                 if SERIAL_READY==1: 
        #                     ser.write("R:0;".encode('utf-8')) #Tells the LLM(low level microcontroller) that we are not seeing any blue object right now
        #                 if DEVELOPING:
        #                     print("Serial: R:0; ")
        #                 serialFlag = 1 #We won't send this "No blue object in vision range" continuosly, we'll just send it once


        if DEVELOPING:  # Provide visual output of the program 
            if SHOW_LINE_ANALYSIS:
                combined_line_masked_frame = cv2.bitwise_or(blue_line_masked_frame, orange_line_masked_frame)
                cv2.imshow("Line Frame", combined_line_masked_frame)

            combined_obstacle_masked_frame = cv2.bitwise_or(green_masked_frame, red_masked_frame)
            cv2.line(combined_obstacle_masked_frame, (int(FRAME_WIDTH/2), 0), (int(FRAME_WIDTH/2), int(FRAME_HEIGHT)), (0, 165, 255), 2)
            cv2.imshow("Obstacle Frame", combined_obstacle_masked_frame)
            

            if TUNE_HSV==1: # We'll only show those segmented staffs only while tuning hsv color values. Otherwise a single unified view will be allowed. 
                if directionSentFlag==-1: 
                    cv2.putText(orange_line_masked_frame, "Clockwise", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                elif directionSentFlag==1: 
                    cv2.putText(blue_line_masked_frame, "Anticlockwise", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

                stackedImages = stackImages(0.6, ([frame, green_masked_frame, red_masked_frame],
                                                    [blue_line_masked_frame, orange_line_masked_frame, combined_line_masked_frame]))
                cv2.imshow("Stacked Frames", stackedImages)


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
            
        



