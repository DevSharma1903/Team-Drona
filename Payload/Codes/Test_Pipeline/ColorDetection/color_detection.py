import cv2
import numpy as np

# =========================
# Configuration
# =========================

INPUT_VIDEO = "input.mov"
AREA_MIN = 200
AREA_MAX = 3000

# HSV thresholds (RPi pipeline)
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])

LOWER_RED2 = np.array([170, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

# Morphology kernel (5-pixel ellipse)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# =========================
# Open Video
# =========================

cap = cv2.VideoCapture(INPUT_VIDEO)

if not cap.isOpened():
    print("Error opening video")
    exit()

# =========================
# Main Loop
# =========================

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --------- RPi Pipeline ---------

    # 1. Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Dual red threshold
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 3. Morphological OPEN then CLOSE
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 4. Contour detection
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy = -1, -1

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # 5. Area filtering
        if AREA_MIN < area < AREA_MAX:

            # 6. Centroid using moments
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Draw result
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.drawContours(frame, [largest], -1, (255, 0, 0), 2)

    # Display results
    cv2.imshow("RPi Pipeline Output", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()