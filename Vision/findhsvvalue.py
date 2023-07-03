import cv2
import numpy as np
from realsense_camera import *

# configuration
camera = RealsenseCamera()

# Create window for trackbars
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

# Create trackbars for HSV lower and upper bound
for i in ["LOW", "HIGH"]:
    v = 0 if i == "LOW" else 255
    for j in ['H', 'S', 'V']:
        cv2.createTrackbar(f"{j}_{i}", "Trackbars", v, 180 if j == 'H' else 255, lambda x: None)

while True:
    ret, bgr_frame, _ = camera.get_frame_stream()
    if not ret:
        break

    # video processing
    frame = cv2.GaussianBlur(bgr_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get trackbar positions
    h_low = cv2.getTrackbarPos("H_LOW", "Trackbars")
    s_low = cv2.getTrackbarPos("S_LOW", "Trackbars")
    v_low = cv2.getTrackbarPos("V_LOW", "Trackbars")

    h_high = cv2.getTrackbarPos("H_HIGH", "Trackbars")
    s_high = cv2.getTrackbarPos("S_HIGH", "Trackbars")
    v_high = cv2.getTrackbarPos("V_HIGH", "Trackbars")

    lower_bound = np.array([h_low, s_low, v_low])
    upper_bound = np.array([h_high, s_high, v_high])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bgr_frame, contours, -1, (0, 255, 0), 2)

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(bgr_frame, (cX, cY), 5, (0, 0, 255), -1)

    cv2.imshow("Frame", bgr_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
print("END")
