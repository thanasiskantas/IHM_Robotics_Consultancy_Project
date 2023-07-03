import pyrealsense2 as rs
import numpy as np
import time
import cv2


class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel RealSense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel RealSense camera is correctly connected")
            return False, None, None

        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        # Create colormap to show the depth of the Objects
        colorizer = rs.colorizer()
        depth_colormap = np.asanyarray(colorizer.colorize(filled_depth).get_data())

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image

    def release(self):
        self.pipeline.stop()


def detect_rectangle_corners(camera):
    ret, bgr_frame, _ = camera.get_frame_stream()
    if not ret:
        return None

    frame = cv2.GaussianBlur(bgr_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Enhance image
    hsv[:, :, 1] = hsv[:, :, 1] * 1.0
    hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)

    light_green = np.array([87, 100, 130])
    dark_green = np.array([90, 255, 255])

    mask = cv2.inRange(hsv, light_green, dark_green)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            # Draw center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(bgr_frame, (cX, cY), 5, (0, 0, 255), -1)

            # Approximate contour to polygon
            epsilon = 0.03 * cv2.arcLength(contour, True)  # 2% error
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                corners = np.array([point[0] for point in approx])
                corners = corners.astype(float)
                return corners

    return None


def order_points_anticlockwise(pts, center):
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    pts = pts[np.argsort(angles)]
    return pts


def get_camera_coordinates(imgpoints):
    leftmotor= [-3.625, 0, 0]
    rightmotor= [3.625, 0, 0]
    t_cameratogripper = np.array([[1.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.99930317, -0.03732524, 3.12703539],
                                  [0.0, 0.03732524, 0.99930317, -7.76569402],
                                  [0.0, 0.0, 0.0, 1.0]])
    intrinsic_camera = np.array(((996.12864831, 0, 648.36399573), (0, 993.66363615, 312.85749898), (0, 0, 1)))
    distortion = np.array((0.30320307, -0.5862013, -0.01298343, 0.00652451, -0.22788748))

    objectpoints = np.array([(1.625, 1.625, 0), (-1.625, 1.625, 0), (-1.625, -1.625, 0), (1.625, -1.625, 0)])

    retval, rvec, tvec = cv2.solvePnP(objectpoints, imgpoints, intrinsic_camera, distortion, cv2.SOLVEPNP_P3P)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    corner_coordinates = []
    dl = []
    dr = []
    for objectpoint in objectpoints:
        homogeneous_objectpoint = np.append(objectpoint, 1)
        camera_corner_coordinates_homo = np.dot(T, homogeneous_objectpoint) #coordinates of corner in camera frame
        gripper_corner_coordinates_homo = np.dot(t_cameratogripper, camera_corner_coordinates_homo) #coordinates of corner in gripper frame
        gripper_corner_coordinates_homo[[1, 2]] = gripper_corner_coordinates_homo[[2, 1]]
        gripper_corner_coordinates = gripper_corner_coordinates_homo[:3] / gripper_corner_coordinates_homo[3]
        if 5 < gripper_corner_coordinates[1] < 7.5:
           gripper_corner_coordinates[1] = 0.01230534* gripper_corner_coordinates[1] ** 3-0.31468437* gripper_corner_coordinates[1] ** 2 + 3.50578319 * gripper_corner_coordinates[1]  - 6.23099935
        if  gripper_corner_coordinates[1] >= 7.5:
            gripper_corner_coordinates[1] = 0.01230534 * gripper_corner_coordinates[1] ** 3 - 0.30468437 * \
                                            gripper_corner_coordinates[1] ** 2 + 3.50578319 * \
                                            gripper_corner_coordinates[1] - 6.23099935
        # if 4.5 < gripper_corner_coordinates[1] < 5.5:
        #     gripper_corner_coordinates[1] -= 0.2
        # elif 5.5 < gripper_corner_coordinates[1] < 6.5:
        #     gripper_corner_coordinates[1] -= 0.0
        # elif 6.5 < gripper_corner_coordinates[1] < 7.5:
        #     gripper_corner_coordinates[1] += 0.2
        # elif 7.5 < gripper_corner_coordinates[1] < 8.5:
        #     gripper_corner_coordinates[1] += 0.25
        # elif 8.5 < gripper_corner_coordinates[1] < 9.5:
        #     gripper_corner_coordinates[1] += 0.28
        # elif 9.5 < gripper_corner_coordinates[1] < 10.5:
        #     gripper_corner_coordinates[1] += 0.3
        # elif 10.5 < gripper_corner_coordinates[1] < 11.3:
        #     gripper_corner_coordinates[1] += 0.35
        # elif 11.3 < gripper_corner_coordinates[1] < 12.5:
        #     gripper_corner_coordinates[1] += 0.6
        corner_coordinates.append(gripper_corner_coordinates)

    ####get centre coordinates
    objectcentre = [0, 0, 0]
    homogeneous_objectcenter = np.append(objectcentre, 1)
    objectcentre_coordinates_homo = np.dot(T, homogeneous_objectcenter)  # coordinates of centre in camera frame
    gripper_centre_coordinates_homo = np.dot(t_cameratogripper,
                                      objectcentre_coordinates_homo)  # coordinates of corner in gripper frame
    gripper_centre_coordinates_homo[[1, 2]] = gripper_centre_coordinates_homo[[2, 1]]
    gripper_centre_coordinates = gripper_centre_coordinates_homo[:3] / gripper_centre_coordinates_homo[3]
    if 5 < gripper_centre_coordinates[1] < 7.5:
        gripper_centre_coordinates[1] = 0.01230534 * gripper_centre_coordinates[1] ** 3-0.31468437 * gripper_centre_coordinates[1] ** 2 + 3.50578319 * gripper_centre_coordinates[1]  - 6.23099935
    if gripper_centre_coordinates[1] >= 7.5:
        gripper_centre_coordinates[1] = 0.01230534 * gripper_centre_coordinates[1] ** 3 - 0.30468437 * \
                                        gripper_centre_coordinates[1] ** 2 + 3.50578319 * gripper_centre_coordinates[
                                            1] - 6.23099935
    #
    # if 4.5 < gripper_centre_coordinates[1] < 5.5:
    #     gripper_centre_coordinates[1] -= 0.2
    # elif 5.5 < gripper_centre_coordinates[1] < 6.5:
    #     gripper_centre_coordinates[1] -= 0.0
    # elif 6.5 < gripper_centre_coordinates[1] < 7.5:
    #     gripper_centre_coordinates[1] +=0.2
    # elif 7.5 < gripper_centre_coordinates[1] < 8.5:
    #     gripper_centre_coordinates[1] += 0.25
    # elif 8.5 < gripper_centre_coordinates[1] < 9.5:
    #     gripper_centre_coordinates[1]+=0.28
    # elif 9.5< gripper_centre_coordinates[1] <10.5:
    #     gripper_centre_coordinates[1] += 0.3
    # elif 10.5 < gripper_centre_coordinates[1] <11.3:
    #     gripper_centre_coordinates[1] += 0.35
    # elif 11.3 < gripper_centre_coordinates[1] < 12.5:
    #     gripper_centre_coordinates[1] += 0.6



    corner_coordinates = np.array(corner_coordinates)
    sorted_corner_coordinates = order_points_anticlockwise(corner_coordinates, gripper_centre_coordinates)

    return sorted_corner_coordinates, np.array(gripper_centre_coordinates)


def main():
    camera = RealsenseCamera()

    # Create a list to store the coordinates from each loop
    coordinates_list = []
    centre_list = []
    while True:
        imgpoints = detect_rectangle_corners(camera)

        if imgpoints is not None and len(imgpoints) == 4:
            coordinates, centre = get_camera_coordinates(imgpoints)
            if coordinates[1, 1] > 20 or coordinates[1, 1] < 3:
                continue
            # Add the coordinates to our list
            coordinates_list.append(coordinates)
            centre_list.append(centre)


            # calculate the average
            if len(coordinates_list) == 5:
                average_coordinates = np.mean(coordinates_list, axis=0)
                average_centre = np.mean(centre_list, axis=0)
                public = average_centre[1]
                print(f"coordinates: {average_coordinates}")
                print(f"centre: {average_centre}")
                print('next loop')
                # Reset the list for the next 10 loops
                coordinates_list = []
                centre_list = []

    camera.release()
    cv2.destroyAllWindows()
    print("END")



if __name__ == '__main__':
    main()
