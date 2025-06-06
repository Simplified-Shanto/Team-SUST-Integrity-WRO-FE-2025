#Objective: During the lap, the car will stop whenever it faces the Red traffic sign on it's way
# and if we exchange that traffic sign with a green one, the car will again resume it's mission

#!/usr/bin/env python 3
import cv2 as cv
import numpy as np
import serial
import time

debug = 0
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
                    imgArray[x][y] = cv.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv.cvtColor( imgArray[x][y], cv.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

ser = serial.Serial('COM17', 115200)
# Whenever the serial communication is established, the arduino resets,
# so we are allowing arduino to have 3 seconds to be completely ready
# for serial communication
time.sleep(3)
# At startup we have a fresh buffer with nothing in it.
ser.reset_input_buffer()
print("Serial is okay:)")



# Traffic sign colors
blue_lower = np.array([95, 157, 84])
blue_upper = np.array([122, 255, 255])

red_lower = np.array([0, 80, 51])
red_upper = np.array([21, 255, 255])


def empty():
    pass
cv.namedWindow("Parameters")
cv.resizeWindow("Parameters",640, 240)
cv.createTrackbar("Threshold1", "Parameters", 20, 255, empty)
cv.createTrackbar("Threshold2", "Parameters", 50, 255, empty)
cv.createTrackbar("Min Area", "Parameters", 20, 200, empty)
# Here "empty" is the name of the function, that will be called each time
# there's a change in the trackbar

frameWidth = 640
frameHeight = 480
cap = cv.VideoCapture(0)
cap.set(3, frameWidth)
cap.set(4, frameHeight)
cap.set(10, 150)
print("Camera Setup done!")


def getContours(img, imgContour, color): # one is the input image, imgContour - is the output image having the contours drawn on it.
    #cv.RETR_EXTERNAL - this parameter is called retrieval method
    # we have 2 main types of retrieval methods
    # the external retrieval method returns the extreme outer corners
    contours, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)


    for contour in contours:
        area = cv.contourArea(contour)
        minArea = cv.getTrackbarPos("Min Area", "Parameters")
        minArea = minArea*1000
        if area > minArea:
            print(area)
            cv.drawContours(imgContour, contour, -1, (255, 0, 255), 7)
            message = " "
            if color=="blue":
                message = "x;" #Stop command for the car
            elif color == "red":
                message = "y;"
            ser.write(message.encode('utf-8'))


while True:
    # Reading images from the webcam stream.
    success, img = cap.read()
    imgHSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    threshold1 = cv.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv.getTrackbarPos("Threshold2", "Parameters")

    ############Blue Traffic Sign Detection############################
    blueMask = cv.inRange(imgHSV, blue_lower, blue_upper)
    imgBlueMasked = cv.bitwise_and(img, img, mask = blueMask)
    imgContourBlue = img.copy() # we'll draw contours on imgContour
    #imgBlur = cv.GaussianBlur(imgBlueMasked, (7, 7), 1)
    imgGrayBlue = cv.cvtColor(imgBlueMasked, cv.COLOR_BGR2GRAY)
    imgCanny = cv.Canny(imgGrayBlue, threshold1, threshold2)
    #Defines the size of the “stamp” for dilation — a bigger kernel thickens edges more.
    kernel = np.ones((5,5))
    # #dilation: Thickens detected edges, connects gaps, makes contours easier to find.
    imgDil = cv.dilate(imgCanny, kernel, iterations = 1)
    getContours(imgDil, imgContourBlue, "blue")

    ############ Red Traffic Sign Detection #########################
    redMask = cv.inRange(imgHSV, red_lower, red_upper)
    imgRedMasked = cv.bitwise_and(img, img, mask=redMask)
    imgContourRed = img.copy()  # we'll draw contours on imgContour
    # imgBlur = cv.GaussianBlur(imgBlueMasked, (7, 7), 1)
    imgGrayRed = cv.cvtColor(imgRedMasked, cv.COLOR_BGR2GRAY)
    imgCanny = cv.Canny(imgGrayRed, threshold1, threshold2)
    # Defines the size of the “stamp” for dilation — a bigger kernel thickens edges more.
    kernel = np.ones((5, 5))
    # #dilation: Thickens detected edges, connects gaps, makes contours easier to find.
    imgDil = cv.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContourRed, "red")

    if debug==1:
        imgStack = stackImages(0.5, ([[img, imgHSV, imgBlueMasked], [imgRedMasked, imgContourBlue, imgContourRed]]))
        cv.imshow("Stacked Images", imgStack)

    if (cv.waitKey(1) & 0xFF)== ord('q'):
        #After we are done with serial communication we'll close it.
        # Because this might cause serial conflict, if another program
        # tries to access this serial port
        print("Serial communication is stopped")
        ser.close()
        break






    '''
    
    

    cv.imshow("Image", img)
    '''


    '''
    #time.sleep(1)
    #message = "l:" + str(counter) + ";"
    message = "x;" #Stopping command for the car.
    ser.write(message.encode('utf-8'))
    print(message)

    '''

# except KeyboardInterrupt:
#     # After we are done with serial communication we'll close it.
#     # Because this might cause serial conflict, if another program
#     # tries to access this serial port
#     print("Serial communication is stopped");
#     ser.close()
'''
Dilation expands the white regions (edges) in the binary image.
👉 Why?
Makes thin edges thicker.
Connects small gaps in edges.
Makes contours easier to find later.
This is especially useful before contour detection, as it helps link broken lines into solid shapes.
'''