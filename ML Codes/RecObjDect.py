import cv2
import numpy as np
import json

# Calibrated or assumed values
KNOWN_WIDTH_CM = 5.0
FOCAL_LENGTH = 700.0

def detect_color(hsv, mask):
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])
    green_lower = np.array([40, 40, 40])
    green_upper = np.array([90, 255, 255])

    red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    red_mask = red_mask1 + red_mask2
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_overlap = cv2.bitwise_and(red_mask, red_mask, mask=mask)
    green_overlap = cv2.bitwise_and(green_mask, green_mask, mask=mask)

    red_pixels = cv2.countNonZero(red_overlap)
    green_pixels = cv2.countNonZero(green_overlap)

    if red_pixels > green_pixels and red_pixels > 100:
        return "Red"
    elif green_pixels > red_pixels and green_pixels > 100:
        return "Green"
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
        green_count = 0
        red_distances = []
        green_distances = []

        red_axis = {"X": 0, "Y": 0}
        green_axis = {"X": 0, "Y": 0}

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
                roi_mask = np.zeros(gray.shape, dtype=np.uint8)
                cv2.drawContours(roi_mask, [approx], -1, 255, -1)

                color = detect_color(hsv, roi_mask)
                distance_cm = estimate_distance(w)
                center_x = x + w // 2
                center_y = y + h // 2

                if color == "Red":
                    red_count += 1
                    red_distances.append(distance_cm)
                    if red_count == 1:
                        red_axis = {"X": center_x, "Y": center_y}
                elif color == "Green":
                    green_count += 1
                    green_distances.append(distance_cm)
                    if green_count == 1:
                        green_axis = {"X": center_x, "Y": center_y}

                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                cv2.putText(frame, f'{color} {distance_cm}cm', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        avg_red_distance = round(sum(red_distances) / len(red_distances), 2) if red_distances else 0
        avg_green_distance = round(sum(green_distances) / len(green_distances), 2) if green_distances else 0

        json_output = {
            "RedObject": {
                "IsRed": red_count > 0,
                "RedCounter": red_count,
                "RedDis": avg_red_distance,
                "RedAxis": red_axis
            },
            "GreenObj": {
                "IsGreen": green_count > 0,
                "GreenCounter": green_count,
                "GreenDis": avg_green_distance,
                "GreenAxis": green_axis
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
