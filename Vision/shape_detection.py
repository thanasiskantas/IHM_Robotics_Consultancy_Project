import cv2
import numpy as np
from realsense_camera import *

# configuration
camera = RealsenseCamera()

while True:
    ret, bgr_frame, _ = camera.get_frame_stream()
    if not ret:
        break

    # video processing
    frame = cv2.GaussianBlur(bgr_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # enhance image
    hsv[:, :, 1] = hsv[:, :, 1] * 1
    hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
    light_green = np.array([79, 100, 123])
    dark_green = np.array([93, 255, 255])

    mask = cv2.inRange(hsv, light_green, dark_green)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bgr_frame, contours, -1, (0, 255, 0), 2)

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            # draw center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(bgr_frame, (cX, cY), 5, (0, 0, 255), -1)

            # approximate contour to polygon
            epsilon = 0.03 * cv2.arcLength(contour, True)  # 2% error
            approx = cv2.approxPolyDP(contour, epsilon, True)


            # if the approximation has 4 points, we assume it's a rectangle
            if len(approx) == 4:

                cv2.drawContours(bgr_frame, [approx], -1, (0, 0, 255), 2)  # draw approximated contour
                for point in approx:
                    cv2.circle(bgr_frame, tuple(point[0]), 4, (0, 0, 255), -1)  # draw corners

    cv2.imshow("Frame", bgr_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
print("END")
