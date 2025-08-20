import cv2

cap = cv2.VideoCapture(0)  # 0 usually means /dev/video0

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    frame = cv2.resize(frame, (640, 480))  # test at lower res
    cv2.imshow("USB Webcam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
