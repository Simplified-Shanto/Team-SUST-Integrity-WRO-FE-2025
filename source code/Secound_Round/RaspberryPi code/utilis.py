import cv2
import numpy as np


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

# img = cv2.imread('Resources/cards.jpg')
# imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#imgStack = stackImages(0.5, ([img, imgGray, img], [img, img, img]))

#cv2.imshow("ImageStack", imgStack)
# imgHor = np.hstack((img, img))
# imgVer = np.vstack((img, img))
# cv2.imshow("Horizontal Stack", imgHor)
# cv2.imshow("Vertical Stack", imgVer)
#cv2.waitKey(0)

def nothing(x):
    pass

def createTrackbarWindow():
            # --- Create General Trackbar Window
    cv2.namedWindow("HSV Trackbars", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("HSV Trackbars", 600, 300)
    cv2.waitKey(100)  # Wait 100ms for the window to draw
    # Create trackbars for general HSV lower and upper bounds
#                                          Intial value of trackbar    Highest value of the trackbar
    cv2.createTrackbar("Green L - H", "HSV Trackbars",          25,                           179,             nothing)
    cv2.createTrackbar("Green L - S", "HSV Trackbars", 135, 255, nothing)
    cv2.createTrackbar("Green L - V", "HSV Trackbars", 50, 255, nothing)
    cv2.createTrackbar("Green U - H", "HSV Trackbars", 55, 179, nothing)
    cv2.createTrackbar("Green U - S", "HSV Trackbars", 255, 255, nothing)
    cv2.createTrackbar("Green U - V", "HSV Trackbars", 255, 255, nothing)
    # HSV tuning trackbars for red objects
    cv2.createTrackbar("Red L - H1", "HSV Trackbars", 0, 179, nothing)
    cv2.createTrackbar("Red L - H2", "HSV Trackbars", 175, 179, nothing)
    cv2.createTrackbar("Red L - S", "HSV Trackbars", 181, 255, nothing)
    cv2.createTrackbar("Red L - V", "HSV Trackbars", 70, 255, nothing)
    cv2.createTrackbar("Red U - H1", "HSV Trackbars", 5, 179, nothing)
    cv2.createTrackbar("Red U - H2", "HSV Trackbars", 179, 179, nothing)
    cv2.createTrackbar("Red U - S", "HSV Trackbars", 255, 255, nothing)
    cv2.createTrackbar("Red U - V", "HSV Trackbars", 255, 255, nothing)
