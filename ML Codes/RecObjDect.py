import cv2
import numpy as np
import json

# Object size and camera focal length (in cm and px)
KNOWN_WIDTH_CM = 10.0
ASPECT_RATIO = 2.0
FOCAL_LENGTH = 700.0

def detect_color(hsv, mask):
    # Red color range (two segments due to hue wrap-around)
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])

    # Blue color range
    blue_lower = np.array([100, 150, 50])
    blue_upper = np.array([140, 255, 255])

    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = red_mask1 + red_mask2
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    red_overlap = cv2.bitwise_and(red_mask, red_mask, mask=mask)
    blue_overlap = cv2.bitwise_and(blue_mask, blue_mask, mask=mask)

    red_pixels = cv2.countNonZero(red_overlap)
    blue_pixels = cv2.countNonZero(blue_overlap)

    if red_pixels > blue_pixels and red_pixels > 100:
        return "Red"
    elif blue_pixels > red_pixels and blue_pixels > 100:
        return "Blue"
    else:
        return "Unknown"

def estimate_distance(perceived_width_px):
    if perceived_width_px == 0:
        return 0
    return round((KNOWN_WIDTH_CM * FOCAL_LENGTH) / perceived_width_px, 2)

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        red_count = 0
        blue_count = 0
        red_distances = []
        blue_distances = []

        red_axis = {"X": 0, "Y": 0}
        blue_axis = {"X": 0, "Y": 0}

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)

            if len(approx) == 4 and area > 1000:
                x, y, w, h = cv2.boundingRect(approx)
                rect_aspect = max(w, h) / min(w, h)
                if not (1.6 <= rect_aspect <= 2.4):
                    continue

                roi_mask = np.zeros(gray.shape, dtype=np.uint8)
                cv2.drawContours(roi_mask, [approx], -1, 255, -1)

                color = detect_color(hsv, roi_mask)
                distance_cm = estimate_distance(max(w, h))
                center_x = x + w // 2
                center_y = y + h // 2

                if color == "Red":
                    red_count += 1
                    red_distances.append(distance_cm)
                    if red_count == 1:
                        red_axis = {"X": center_x, "Y": center_y}
                elif color == "Blue":
                    blue_count += 1
                    blue_distances.append(distance_cm)
                    if blue_count == 1:
                        blue_axis = {"X": center_x, "Y": center_y}

                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                cv2.putText(frame, f'{color} {distance_cm}cm', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        avg_red_distance = round(sum(red_distances) / len(red_distances), 2) if red_distances else 0
        avg_blue_distance = round(sum(blue_distances) / len(blue_distances), 2) if blue_distances else 0

        json_output = {
            "RedObject": {
                "IsRed": red_count > 0,
                "RedCounter": red_count,
                "RedDis": avg_red_distance,
                "RedAxis": red_axis
            },
            "BlueObject": {
                "IsBlue": blue_count > 0,
                "BlueCounter": blue_count,
                "BlueDis": avg_blue_distance,
                "BlueAxis": blue_axis
            }
        }

        print(json.dumps(json_output, indent=4))
        cv2.imshow("Box Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
