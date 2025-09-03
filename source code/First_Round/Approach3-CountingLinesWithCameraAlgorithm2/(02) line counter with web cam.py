# --- Configuration ---
SERIAL_READY = 1 #Whether a serial device is connected or not
CAM_TYPE = 0 # 0  = Raspcamera, 1  = webcam. 
CAMERA_INDEX = 0    # Select which cam will be used  #1 - laptop's camera #0 - micropack webcam 
COM_PORT = 4
MACHINE = 0  # 0 = WINDOWS, 1 = LINUX OS, (Raspberry pie)
TUNE_HSV = 0 # whether we want to tune the hsv color values for different image elements. 
#!/usr/bin/env python3
DEVELOPING   = 1 # The code is in development mode, and we'll show processed images at different stages, 
                 # otherwise, there'll be no ui output of the code thus we can run it headless on startup i
                 # in raspberry pie. 


THRESHOLD_AREA = 1000
lineInterval = 1000 # The interval between counting consecutive lines. 
if MACHINE == 0 and CAM_TYPE==0: 
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

# We'll count lines based on the blue line only - because, there's possiblity of overlapping for red and orange line. Later, we may count both lines with their relative order for more precise stopping at the position. 
# Right now we'll only focus on stopping after completing 3 full laps. 

blue_lower = np.array([71, 203 , 0 ])
blue_upper = np.array([179, 255, 255 ])

# orange_lower = np.array([0, 127, 163 ])
# orange_upper = np.array([47, 255, 255 ])

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

line_count = -1 # The counting hasn't begun yet. 
last_time = time.time() * 1000 # Getting the total execution time in millisecond

blue_line_lower_bound = np.array([99, 40 , 90 ])
blue_line_upper_bound = np.array([135, 255, 255 ])

def nothing(x):
    pass

if TUNE_HSV==1 and DEVELOPING==1:
    # --- Create General Trackbar Window
    cv2.namedWindow("HSV Trackbars", cv2.WINDOW_NORMAL) #WINDOW_NORMAL is a flag that let's user resize the window, by default its, cv2.AUTOSIZE, which makes the window fit to the size of the content and it is unresizable. 
    cv2.resizeWindow("HSV Trackbars", 600, 600)
    cv2.waitKey(100)  # Wait 100ms for the window to draw
    # Create trackbars for general HSV lower and upper bounds

    cv2.createTrackbar("Blue Line L_H", "Line HSV trackbars",blue_line_lower_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line L_S", "Line HSV trackbars",blue_line_lower_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line L_V", "Line HSV trackbars",blue_line_lower_bound[2], 255, nothing )
    cv2.createTrackbar("Blue Line U_H", "Line HSV trackbars",blue_line_upper_bound[0], 179, nothing )
    cv2.createTrackbar("Blue Line U_S", "Line HSV trackbars",blue_line_upper_bound[1], 255, nothing )
    cv2.createTrackbar("Blue Line U_V", "Line HSV trackbars",blue_line_upper_bound[2], 255, nothing )

if SERIAL_READY:
    message = 'r;' #Suggests to the esp32 that the raspberry pie is ready for image processing 
    ser.write(message.encode('utf-8'))
    time.sleep(1)  


while True:
    if ser.in_waiting > 0:  # If there's some message from Arduino
        command = ser.readline().decode('utf-8').strip()  # Read line & strip newline/spaces
        if DEVELOPING == 1:
            print("Raw command = ", command)
        # Case 1: simple one-letter command like 'r' or 'd'
        if command == "r":   # The lap is starting via button press, so start counting lines
            line_count = 0
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

    if CAM_TYPE==1:
        success, frame = cap.read()
    elif CAM_TYPE==0: 
        frame = picam2.capture_array()

    
    #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # #cv2.imshow("Original", frame)
    frame = frame[100:400, 150:590] #cropping the image to extract only useful part
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_blue = cv2.inRange(hsv, blue_line_lower_bound, blue_line_upper_bound)
    #mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
    
    blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #This basically measures, which color of threshold area do we get first
    for contour_index, contour in enumerate(blue_contours): 
        area = cv2.contourArea(contour)
       # if DEVELOPING==1:
               # print("blue = ", area)
        if (current_time - last_time > lineInterval) and (cv2.contourArea(contour) > THRESHOLD_AREA) and line_count!=-1:
            line_count = line_count + 1
            last_time = current_time

    if line_count==12 and SERIAL_READY==1: 
        message = 'x;' #Commands to stop the car. 
        time.sleep(stopDelay/1000)  # Waiting a bit to reach the center fo the tunnel. 
        ser.write(message.encode('utf-8'))
        #time.sleep(1)
        line_count = -1  # We won't count lines until a new lap is started by pressing the button

    
    # for cntour_index, contour in enumerate(orange_contours): 
    #     area = cv2.contourArea(contour)
    #     print("orange = ", area)
    #     if area > thresholdArea:
    #         message = 'o;'
    #         ser.write(message.encode('utf-8'))
    #         selectSetPoint = 0
    #         time.sleep(1)

        #cv2.imshow("orange_mask", mask_orange)
    # else: 
    #     time.sleep(0.01)
    #     if ser.in_waiting > 0: 
    #         command = ser.readline().decode('utf-8')
    #         print(command)
    #         selectSetPoint = 1
    if DEVELOPING==1: 
        masked_image = cv2.bitwise_and(frame, frame, mask = mask_blue); 
        cv2.putText(masked_image,f"lines = {line_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        cv2.imshow("blue_mask", masked_image); 


    if DEVELOPING ==1 and TUNE_HSV == 1:
        bl_l_h = cv2.getTrackbarPos("Blue Line L_H", "Line HSV trackbars") #bl_l_h = blue line lower hue
        bl_l_s = cv2.getTrackbarPos("Blue Line L_S", "Line HSV trackbars")
        bl_l_v = cv2.getTrackbarPos("Blue Line L_V", "Line HSV trackbars")
    
        bl_u_h = cv2.getTrackbarPos("Blue Line U_H", "Line HSV trackbars") 
        bl_u_s = cv2.getTrackbarPos("Blue Line U_S", "Line HSV trackbars")
        bl_u_v = cv2.getTrackbarPos("Blue Line U_V", "Line HSV trackbars")
        blue_line_lower_bound = np.array([bl_l_h, bl_l_s , bl_l_v ])
        blue_line_upper_bound = np.array([bl_u_h, bl_u_s,  bl_u_v ])


    # Press 'q' to quit
    if DEVELOPING==1: 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            if SERIAL_READY:
                ser.close()
            break
if DEVELOPING==1:
    cv2.destroyAllWindows()
