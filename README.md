# In Hand Manipulation (IHM) Project at Imperial College London

By Athanasios Kantas, Qiyang Yan, Sean Xiao, Nick Zheng, Alexandria Perkin

## Table of Contents

- [Introduction](#introduction)
- [Implementation](#Implementation)
- [Materials](#Materials)
- [Ethical Consequences](#Ethical_Consequences)
- [Sustainability](#Sustainability)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)


## Introduction

The problem that the team was tasked to approach was that of In-Hand Manipulation (IHM). Rotation and of objects using the traditional approach of Pick-Twist-Place is very spatially inefficient. This problem is solved by the use of IHM grippers that allow for sliding and rotating with minimal arm movement. The way the team decided to showcase IHM using these grippers, was by doing a Pick-IHM-Place task using simple toy blocks.

## Implementation

To implement such a task, the team decided to integrate three components: The gripper, the arm, and computer vision. The gripper is responsible for the IHM, the arm is responsible for the pick-place task, and the computer vision works with both to achieve error detection and correction.

After all three components functioned correctly individually, they were integrated in a way like seen in Figure (*PROVIDE LINK FOR PICTURE*).

More specifically, different platforms were used to control each section of the project. The gripper is controlled using MATLAB, the arm is controlled using python, and computer vision processing is done in C++. The high-level code which combines all three is written in Python, allowing each section to communicate using ROS topics. An example of this inter-platform communication is displayed in Figure (*PROVIDE LINK FOR PICTURE*).\\


## Computer Vision

This part aims to develop a vision system to accurately detect and track an object's coordinates using a depth camera. The goal is to obtain the object's coordinates in the gripper's coordinate frame, which can then be used in a gripper control algorithm. The vision system uses an Intel RealSense D435i depth camera mounted on an adjustable base with an adjustable angle to capture the entire field of view of the gripper. The system processes images and applies algorithms to identify and track the object of interest.


### Hardware 

![Vision hardware setup](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/d2c591755910f07e84d46e6dd812f25e6b57b41a/Vision/cameraandthemount.png)

#### Hardware design history 

1. The initial un-adjustable base design didn't allow the RGB camera to be centered due to its left-sided placement on the camera.
2. The un-adjustable angle didn't include the whole gripper in the image, regardless of its position.
3. The final design incorporates an adjustable base and angle. This design allows the camera to capture the entire field of view of the gripper, keeping the gripper centered at the start position.

### Software 

#### Software design history

1. The initial approach used an ArUco marker for object detection, which proved to be accurate and robust. However, this method required different markers for different objects, which is not desirable for shape sorting.
   
2. The second approach used HSV colour space thresholding. This technique allows object identification without changing the object's characteristics. However, the HSV threshold values must be calibrated to accommodate different lighting conditions.

#### Detailed Implementation 

##### Image processing

![Vision hardware setup](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/d2c591755910f07e84d46e6dd812f25e6b57b41a/Vision/hsvflowchart.PNG)


Images are processed using Gaussian blur filtering before thresholding to reduce high-frequency noise. Post thresholding, morphological operations, and median blur filters are applied to further reduce noise and smooth the image's edges.

##### Coordinates calculation


![Vision hardware setup](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/d2c591755910f07e84d46e6dd812f25e6b57b41a/Vision/solvepnp.jpg)

OpenCV's contour detection function identifies the contours in the image. A bounding box is drawn around the largest contour, taken as the object of interest. The center of the object is calculated as the geometric center of the bounding box. The SolvePnP functions from OpenCV are used to get the coordinates of the object in 3D relative to the camera coordinate frame. The theory is shown in the figure above. The transformation matrix, measured in the CAD file, transforms the coordinates from the camera frame to the gripper frame. Specifically, the transformation is 0 on the x-axis, -28.35 on the y-axis, and 78.77 on the z-axis, with a rotation of 122.56 degrees anticlockwise around the x-axis. These values are project-specific and depend on the camera mount design.

### Performance and Recommendations

The system, using a camera with FPS30 and 1280X720 resolution, updates the object's center coordinates in each frame, providing real-time tracking. The system performs optimally in well-lit environments with uniquely colored objects. However, changes in lighting conditions can affect HSV values, necessitating occasional calibration. 



### Usage

#### Package Installation
 Before using the project, install the necessary Python packages: 
 pyrealsense2 (if using Intel RealSense camera);
 numpy;
opencv-python

#### Camera Calibration

Clone the project from https://github.com/niconielsen32/CameraCalibration.git and prepare the images containing the chessboard taken by the camera intended to use in the project. Collect the camera's intrinsic and distortion coefficients and adjust these values in the get_camera_coordinates function in the [finalvision_hsv.py](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/Vision/finalvision_hsv.py).

#### HSV Value
Run the [findhsvvalue.py](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/Vision/findhsvvalue.py) and drag the track bar to find a range of HSV values in  [finalvision_hsv.py](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/Vision/finalvision_hsv.py) that can isolate the desired colour block. 


## Gripper Control
This algorithm is designed to utilise the finger to realise the in-hand manipulation for various sizes of objects, including sliding and rotation, with the goal of minimising the error. Now the algorithm is being tested on the following shapes: square cube, hexagon prism, octagon prism. 

[trajectory_final.m](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/d2c591755910f07e84d46e6dd812f25e6b57b41a/Gripper%20Trajectory%20Generation/trajectory_final.m) generates the trajectory consists sliding and rotation based on the start pose and end pose entered by the user, you could simulate the generated trajectory using this file. The rotation within the trajectory only shows the start and end pose of the rotation without trajectory in-between being shown, the sliding trajectory of the center of the object is shown with black arcs.

### Simulation

In addition, the folder [Gripper Simulation](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/tree/bfcf90439357cf4e225c7dbe434d14aee228815c/Gripper%20Simulation) produces a simulation process carried out using Simulink to observe the motion of objects within the gripper. A spatial contact force block is introduced to simulate various friction forces. This simulation aided in understanding the behavior of the system and allowed for the identification of preferred rotation start poses. 

With the help of simulation, six types of statuses within the gripper are observed for the cube with four of them being preferred rotate start pose. However, at the later stage of testing, the friction status of two fingers doesn’t show any impact on the pose of the cube, the pose is found to be purely dependent on the clockwise or anticlockwise rotation of the fingers. For example, during the slide, if the object moves from the left side to the right side with fingers rotated clockwise, regardless of the friction pad, the object ends at the pose shown in Figure 1.

### Sliding

Regarding sliding, an important observation is that any two points within the gripper moving range could be reached with three circular arcs. Therefore, for sliding movements, a mathematical model consisting of three linked arcs is introduced, allowing the gripper to select the combination with the shortest arc length using gradient descent, thereby minimizing the error introduced by friction switches. The motor angles required for sliding movements are provided in the equations.

![Sliding Mathematical Model](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_1.png)


### Rotation

A mathematical model for rotation is then presented, providing equations for clockwise and anticlockwise rotations. 

$$R2L_{left motor} = π - cos^{-1}(\frac{d_L^2 + (W/2)^2 - (d/2)^2}{d_L * (W/2)}) - cos^{-1}(\frac{l_1^2 + 2 * L^2 - d_R^2}{4 * l_1 * L^2}) - cos^{-1}(\frac{d_L^2 + l_1^2 - d^2}{2 * d_L * l_1}) - θ_L $$

To compensate for the small angle in the undesired rotation prep-pose, an extra angle calculation is introduced, effectively removing the introduced error. Furthermore, a control strategy is designed to extend the limited rotation range of the gripper, allowing rotations from -180 to 180.

![Rotation Mathematical Model](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_2.png)

### Control Strategy
Another problem was the change in the thickness of the finger when changing the level of friction. This was solved by introducing a transitional layer between adjacent moves to ensure smooth transitions and precise open-loop control. The corresponding function that will be used in the integration could be found here [getTrajectory_improved.m](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/d2c591755910f07e84d46e6dd812f25e6b57b41a/IHM%20Integrated%20Control/getTrajectory_improved.m) 

Additionally, closed-loop control is implemented, incorporating a vision subsystem that provides real-time localization and feedback coordinates. More information regarding the vision subsystem can be found in the dedicated section of the report. 

![Control Strategy](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_3.png)


## Materials

Selecting the right high friction material for the surface of a gripper's finger is crucial for achieving rotation. We selected several materials that could be used as the high friction material on the surface of the gripper's finger. After testing each one, we found that the material with the highest friction was Dycem as the surface of Dycem is tacky, allowing it to adhere to objects and provide a high coefficient of friction.


## Ethical Consequences

The primary ethical concern is ensuring the safety of humans that come into contact with the gripper and robotic arm. The UR5e robotic arm, that the gripper is attached to, is equipped with built-in force torque sensors that allow it to detect and respond to unexpected contact or collisions. When the robot comes into contact with an object or a person, it can quickly sense the force and halt its movements to prevent harm.

The introduction of robotic systems, including gripper technology, has the potential to automate certain tasks traditionally performed by humans. This can lead to job displacement, as some roles may no longer require human labour. The variable friction gripper is compact and able to perform manipulation of objects in confined spaces, unlike more traditional methods. This may lead to the replacement of workers previously used to do these tasks, for example, item pickers in a warehouse.

The gripper is equipped with a realsense camera which is used only to determine the location of the object being manipulated. No data from the camera is stored and so there are no privacy concerns.


## Sustainability

Dycem loses its stickiness over time however washing it and removing any debris sticking to the surface will restore it. Over longer periods of time the dycem gripper pads may need to be replaced to maintain the high coefficient of friction provided by the material. The gripper is designed so the gripper pads can easily be unscrewed and replaced with new ones, without needing to replace any other part of the gripper, reducing waste.

The variable friction gripper reduces the movement of the robot arm when performing manipulation, minimizing energy consumption, which translates to reduced environmental impact.

## Installation

Provide step-by-step instructions on how to install your project. Include any dependencies or prerequisites that need to be set up.

```shell
$ git clone https://github.com/your-username/your-repo.git
$ cd your-repo
$ npm install
