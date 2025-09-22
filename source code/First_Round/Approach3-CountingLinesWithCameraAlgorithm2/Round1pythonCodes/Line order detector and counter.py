# --- Configuration ---
#-----Full forms------------------
#LLMC - Low level microcontroller
#SBC - Single Board Computer

# Uncomment the following lines of code to know which python interepreter is currently being used by the script. 
# import sys
# print(sys.executable)
# print(sys.version)

#!/usr/bin/env python3

# --- Configuration ---
DEVELOPING   = 1 # The code is in development mode, and we'll show processed images at different stages, 
                 # otherwise, there'll be no ui output of the code thus we can run it headless on startup i
                 # in raspberry pie. 
CAM_TYPE = 0 # 0  = Raspicamera, 1  = webcam. 
CAMERA_INDEX = 0    # Select which cam will be used  #1 - laptop's camera #0 - micropack webcam 
COM_PORT = 4
MACHINE = 1  # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
TUNE_HSV = 0 # whether we want to tune the hsv color values for different image elements. 
SERIAL_READY = 1 #Whether a serial device is connected or not


MIN_LINE_AREA = 5000
lineInterval = 1000 # The interval between counting consecutive lines. 
if MACHINE == 1 and CAM_TYPE==0: 
    from picamera2 import Picamera2
import time
import cv2
import numpy as np
import serial
import os


if SERIAL_READY==1 and MACHINE == 1:
    ser = serial.Serial("/dev/esp32_serial", 115200, timeout = 1)
    time.sleep(2)
    # At startup we have a fresh buffer with nothing in it. 
    ser.reset_input_buffer()
elif SERIAL_READY==1 and MACHINE ==0: 
    ser = serial.Serial(f'COM{COM_PORT}', 115200, timeout = 1.0)
    time.sleep(2)
    ser.reset_input_buffer()


FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# # --- Initialize camera ---
if MACHINE == 0:  # Windows
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

elif MACHINE==1:             # Linux / Raspberry Pi
    if CAM_TYPE==0: # Pi camera 
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
    elif CAM_TYPE==1:  # USB webcam. 
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

orange_line_count = -1 # The counting hasn't begun yet. 
orange_line_timer = time.time() * 1000 # Getting the total execution time in millisecond
directionSentFlag = 0 # Whether we've reported the direction of the round to the LLM

# We'll count lines based on the orange line only - because, there's possiblity of exposure to abudant blue region inside the small boundry and in the outer boundry. Later, we may count both lines with their relative order for more precise stopping at the position. 

# Nighttime condition 1 

# blue_line_lower_bound = np.array([99, 40 , 90 ])
# blue_line_upper_bound = np.array([135, 255, 255 ])

# orange_line_lower_bound = np.array([174, 102, 14 ])
# orange_line_upper_bound = np.array([179, 170, 255 ])

# Nighttime condition 2 

blue_line_lower_bound = np.array([64, 82 , 0 ])
blue_line_upper_bound = np.array([137, 255, 255 ])

orange_line_lower_bound = np.array([0, 62, 0 ])
orange_line_upper_bound = np.array([69, 255, 255 ])



#Daytime condition 1 

# blue_line_lower_bound = np.array([101, 82 , 00 ])
# blue_line_upper_bound = np.array([167, 255, 255 ])

# orange_line_lower_bound = np.array([155, 127, 0 ])
# orange_line_upper_bound = np.array([179, 255, 255 ])



def nothing(x):
    pass

if TUNE_HSV==1 and DEVELOPING==1:
    # --- Create General Trackbar Window
    cv2.namedWindow("Line HSV trackbars", cv2.WINDOW_NORMAL) #WINDOW_NORMAL is a flag that let's user resize the window, by default its, cv2.AUTOSIZE, which makes the window fit to the size of the content and it is unresizable. 
    cv2.resizeWindow("Line HSV trackbars", 600, 600)
    cv2.waitKey(100)  # Wait 100ms for the window to draw
    # Create trackbars for general HSV lower and upper bounds

    cv2.createTrackbar("Blue Line L_H", "Line HSV trackbars",blue_line_lower_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line L_S", "Line HSV trackbars",blue_line_lower_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line L_V", "Line HSV trackbars",blue_line_lower_bound[2], 255, nothing )
    cv2.createTrackbar("Blue Line U_H", "Line HSV trackbars",blue_line_upper_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line U_S", "Line HSV trackbars",blue_line_upper_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line U_V", "Line HSV trackbars",blue_line_upper_bound[2], 255, nothing )


    cv2.createTrackbar("Orange Line L_H", "Line HSV trackbars",orange_line_lower_bound[0], 179, nothing )
    cv2.createTrackbar("Orange Line L_S", "Line HSV trackbars",orange_line_lower_bound[1], 255, nothing )
    cv2.createTrackbar("Orange Line L_V", "Line HSV trackbars",orange_line_lower_bound[2], 255, nothing )
    cv2.createTrackbar("Orange Line U_H", "Line HSV trackbars",orange_line_upper_bound[0], 179, nothing )
    cv2.createTrackbar("Orange Line U_S", "Line HSV trackbars",orange_line_upper_bound[1], 255, nothing )
    cv2.createTrackbar("Orange Line U_V", "Line HSV trackbars",orange_line_upper_bound[2], 255, nothing )

if SERIAL_READY:
    message = 'r;' #Suggests to the esp32 that the raspberry pie is ready for image processing 
    ser.write(message.encode('utf-8'))
    time.sleep(1)  

while True:
    if SERIAL_READY: 
        if ser.in_waiting > 0:  # If there's some message from LLMC
            command = ser.readline().decode('utf-8').strip()  # Read line & strip newline/spaces
            if DEVELOPING == 1:
                print("Raw command = ", command)
            # Case 1: simple one-letter command like 'r' or 'd'
            if command == "r":   # The lap is starting via button press, so start counting lines
                orange_line_count = 0
                directionSentFlag = 0  
                if DEVELOPING: print("Lap started (reset line count).")

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

    current_time = time.time() * 1000

    if DEVELOPING == 1 or orange_line_count!=-1: # do all the image processing, either if we are developing the code, or we are running a lap. 
        if CAM_TYPE==1:
            success, frame = cap.read()
        elif CAM_TYPE==0: 
            frame = picam2.capture_array()
        
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # #cv2.imshow("Original", frame)
        frame = frame[100:400, 150:590] #cropping the image to extract only useful part
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask_blue = cv2.inRange(hsv, blue_line_lower_bound, blue_line_upper_bound)
        mask_orange = cv2.inRange(hsv , orange_line_lower_bound, orange_line_upper_bound)
    
        blue_line_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_line_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if orange_line_count!=-1: # We'll only do the following processes when orange_line_count is set to zero by pressing the game start button in the vehicle and receiving serial command 'r' from the LLMC. The reason is avoiding early count of the lines by environmental noise before the round has started. 
            max_blue_line_area = 0
            for contour_index, contour in enumerate(blue_line_contours): 
                area = cv2.contourArea(contour)
                #print(f"Blue Area = {area}"); 
                max_blue_line_area = max(max_blue_line_area, area)
                           
            max_orange_line_area = 0
            for cntour_index, contour in enumerate(orange_line_contours): 
                area = cv2.contourArea(contour)
                #print(f"Orange Aera = {area}"); 
                max_orange_line_area = max(max_orange_line_area, area)

     
            if directionSentFlag == 0:
                if (max_orange_line_area > MIN_LINE_AREA) and (max_orange_line_area > max_blue_line_area): 
                        if SERIAL_READY:
                            ser.write("o;".encode('utf-8'))
                        if DEVELOPING: 
                            print("Serial: o;")
                        directionSentFlag = -1 # Round is clockwise
                elif (max_blue_line_area > MIN_LINE_AREA) and (max_blue_line_area > max_orange_line_area):
                        if SERIAL_READY:
                            ser.write("b;".encode('utf-8'))
                        if DEVELOPING:
                            print("Serial: b;")
                        directionSentFlag = 1  # Round is anticlockwise
            
            # Line counting and checking for lap completion 
            if (current_time - orange_line_timer > lineInterval) and (max_orange_line_area >= 500): # We are considering a very small area for line counting compared to the   MIN_LINE_AREA used for line order detection , because whihle moving really fast, not all contours are visible, so we will have to detect for very small contour area 
                orange_line_timer = current_time
                orange_line_count+=1
                if orange_line_count==12:
                    if DEVELOPING==1: 
                        print("3 laps done. Waiting for RESET command"); 
                    if SERIAL_READY==1: 
                        message = 'x;' #Commands to stop the motor only 
                        time.sleep(stopDelay/1000)  # Waiting a bit to reach the center fo the tunnel. 
                        ser.write(message.encode('utf-8'))
                        time.sleep(2)  #Wating for 2 second before sending the steering off command
                        message = 'z;' #Command to stop the lapse completely
                        ser.write(message.encode('utf-8'))
                    orange_line_count = -1  # We won't count lines until a new lap is started by pressing the button
                    roundDirection = 0

        
    if DEVELOPING==1: 
        orange_line_masked_frame = cv2.bitwise_and(frame, frame, mask = mask_orange)
        blue_line_masked_frame = cv2.bitwise_and(frame, frame, mask = mask_blue)
        combined_line_mask = cv2.bitwise_or(blue_line_masked_frame, orange_line_masked_frame)
        
        cv2.putText(combined_line_mask,f"lines = {orange_line_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        if directionSentFlag==-1: 
            cv2.putText(combined_line_mask, "Clockwise", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
        elif directionSentFlag==1: 
            cv2.putText(combined_line_mask, "Anticlockwise", (50, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imshow("combined_mask", combined_line_mask); 


        if TUNE_HSV == 1:
            # Only show individual line frames when we are tuning the hsv values. 
            cv2.imshow("orange line mask", orange_line_masked_frame)
            cv2.imshow("blue line mask", blue_line_masked_frame)
            bl_l_h = cv2.getTrackbarPos("Blue Line L_H", "Line HSV trackbars") #bl_l_h = blue line lower hue
            bl_l_s = cv2.getTrackbarPos("Blue Line L_S", "Line HSV trackbars")
            bl_l_v = cv2.getTrackbarPos("Blue Line L_V", "Line HSV trackbars")
        
            bl_u_h = cv2.getTrackbarPos("Blue Line U_H", "Line HSV trackbars") 
            bl_u_s = cv2.getTrackbarPos("Blue Line U_S", "Line HSV trackbars")
            bl_u_v = cv2.getTrackbarPos("Blue Line U_V", "Line HSV trackbars")

            blue_line_lower_bound = np.array([bl_l_h, bl_l_s , bl_l_v ])
            blue_line_upper_bound = np.array([bl_u_h, bl_u_s,  bl_u_v ])

            ol_l_h = cv2.getTrackbarPos("Orange Line L_H", "Line HSV trackbars") #bl_l_h =  orange line lower hue
            ol_l_s = cv2.getTrackbarPos("Orange Line L_S", "Line HSV trackbars")
            ol_l_v = cv2.getTrackbarPos("Orange Line L_V", "Line HSV trackbars")
        
            ol_u_h = cv2.getTrackbarPos("Orange Line U_H", "Line HSV trackbars") 
            ol_u_s = cv2.getTrackbarPos("Orange Line U_S", "Line HSV trackbars")
            ol_u_v = cv2.getTrackbarPos("Orange Line U_V", "Line HSV trackbars")
            
            orange_line_lower_bound = np.array([ol_l_h, ol_l_s , ol_l_v ])
            orange_line_upper_bound = np.array([ol_u_h, ol_u_s,  ol_u_v ])
            
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            if SERIAL_READY:
                ser.close()
            break

if DEVELOPING==1:
    cv2.destroyAllWindows()
