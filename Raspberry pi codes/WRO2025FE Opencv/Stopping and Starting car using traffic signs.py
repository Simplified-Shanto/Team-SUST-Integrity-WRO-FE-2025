#Objective: During the lap, the car will stop whenever it faces the Red traffic sign on it's way
# and if we exchange that traffic sign with a green one, the car will again resume it's mission

#!/usr/bin/env python 3
import cv2
import cv2 as cv
import numpy as np
import serial
import time
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

# ser = serial.Serial('COM4', 115200)
# # Whenever the serial communication is established, the arduino resets,
# # so we are allowing arduino to have 3 seconds to be completely ready
# # for serial communication
# time.sleep(3)
# # At startup we have a fresh buffer with nothing in it.
# ser.reset_input_buffer()
# print("Serial is okay:)")

# Traffic sign colors
blue_lower = np.array([95, 157, 84])
blue_upper = np.array([122, 255, 255])

red_lower = np.array([0, 142, 40])
red_upper = np.array([179, 255, 250])


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


def getContours(img, imgContour): # one is the input image, imgContour - is the output image having the contours drawn on it.
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
            # # peri is the perimeter of the contour
            # peri = cv2.arcLength(contour, True) # True indicates that the contour is closed
            # #Purpose of approxPolyDP: Instead of hundreds of small points, you get a few key points describing the shape.
            # approx = cv2.approxPolyDP(contour, 0.02*peri, True)
            # print(len(approx))

# try:
while True:
    # Reading images from the webcam stream.
    success, img = cap.read()
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blueMask = cv2.inRange(imgHSV, blue_lower, blue_upper)
    imgBlueMasked = cv2.bitwise_and(img, img, mask = blueMask)

    imgContour = img.copy() # we'll draw contours on imgContour
    imgBlur = cv.GaussianBlur(imgBlueMasked, (7, 7), 1)
    imgGray = cv.cvtColor(imgBlueMasked, cv.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2","Parameters")
    imgCanny = cv.Canny(imgGray, threshold1, threshold2)
    #Defines the size of the “stamp” for dilation — a bigger kernel thickens edges more.
    kernel = np.ones((5,5))
    # #dilation: Thickens detected edges, connects gaps, makes contours easier to find.
    imgDil = cv.dilate(imgCanny, kernel, iterations = 1)

    getContours(imgDil, imgContour)
    # imgStack = stackImages(0.8, ([[img, imgBlur, imgGray, imgCanny], [imgDil, imgContour, imgDil, imgDil]]))
    imgStack = stackImages(0.5, ([[img, imgHSV, imgBlueMasked], [imgContour, imgContour, imgContour]]))
    cv.imshow("Stacked Images", imgStack)
    if (cv.waitKey(1) & 0xFF)== ord('q'):
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