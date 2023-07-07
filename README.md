# In Hand Manipulation (IHM) Project at Imperial College London

By Athanasios Kantas, Qiyang Yan, Sean Xiao, Nick Zheng, Alexandria Perkin  
Supervisor: Dr Ad Spiers

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

Welcome to the repository for the 3rd-year consultancy project conducted in collaboration with the Imperial College London Manipulation and Touch Lab. The objective of this project was to use a robot arm and a novel variable friction gripper, to perform a toy assembly task with In Hand Manipulation. This project serves as a proof of concept for a potential production line application. This documentation will provide you with a comprehensive understanding of the project and its components, enabling you to take over its development.


## Implementation

To implement such a task, the team decided to integrate three components: The gripper, the arm, and computer vision. The gripper is responsible for the IHM, the arm is responsible for the pick-place task, and the computer vision works with both to achieve error detection and correction.

After all three components functioned correctly individually, they were integrated in a way like seen in Figure (*PROVIDE LINK FOR PICTURE*).

More specifically, different platforms were used to control each section of the project. The gripper is controlled using MATLAB, the arm is controlled using python, and computer vision processing is done in C++. The high-level code which combines all three is written in python, allowing each section to communicate using ROS topics. An example of this inter-platform communication is displayed in Fgure (*PROVIDE LINK FOR PICTURE*).\\

###Gripper Control
Explaining how gripper control works
This algorithm is designed to utilise the finger to realise the in-hand manipulation for various sizes of objects, including sliding and rotation, with the goal of minimising the error. Now the algorithm is being tested on the following shapes: square cube, hexagon prism, octagon prism. 

Regarding sliding, an important observation is that any two points within the gripper moving range could be reached with three circular arcs. Therefore, for sliding movements, a mathematical model consisting of three linked arcs is introduced, allowing the gripper to select the combination with the shortest arc length using gradient descent, thereby minimizing the error introduced by friction switches. The motor angles required for sliding movements are provided in the equations.

In addition, the file ____ produces a simulation process carried out using Simulink to observe the motion of objects within the gripper. A spatial contact force block is introduced to simulate various friction forces. This simulation aided in understanding the behaviour of the system and allowed for the identification of preferred rotation start poses. A mathematical model for rotation is then presented, providing equations for clockwise and anticlockwise rotations.

With the help of simulation, six types of statuses within the gripper are observed for the cube with four of them being preferred rotate start pose. However, at the later stage of testing, the friction status of two fingers doesn’t show any impact on the pose of the cube, the pose is found to be purely dependent on the clockwise or anticlockwise rotation of the fingers. For example, during the slide, if the object moves from the left side to the right side with fingers rotated clockwise, regardless of the friction pad, the object ends at the pose shown in Figure 1.

To compensate for the small angle in the undesired rotation prep-pose, an extra angle calculation is introduced, effectively removing the introduced error. Furthermore, a control strategy is designed to extend the limited rotation range of the gripper, allowing rotations from -180 to 180.

Another problem was the change in the thickness of the finger when changing the level of friction. This was solved by introducing a transitional layer is introduced between adjacent moves to ensure smooth transitions and precise open-loop control. Additionally, closed-loop control is implemented, incorporating a vision subsystem that provides real-time localization and feedback coordinates. More information regarding the vision subsystem can be found in the dedicated section of the report. 



###Computer Vision Processing
Explaining how computer vision works

## Prerequisites

- Linux machine with ROS, Moveit, Rviz and Universal Robot driver (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver). Moveit uses python interface to build simulation environment, get end effector poses and plan/execute trajectory while Rviz allows the user to view the planned path via simulation. 
- For usage that requires remote control for the UR5E: MATLAB with ROS Toolbox
- Realsense driver... opencv_contrib version??

## How to use

### UR5e
-  Start the UR5e and open External control program

### ROS in linux machine
- Enter the ROS workspace where you installed the driver in
```
    source /opt/ros/noetic(your version)/setup.bash
    cd ~/catkin_wd（your workspace folder)
    source devel/setup.bash
    
```
- For Simulation only
```
  roslaunch ur5e_moveit_config demo.launch
```
- For real robot:
  1. Start ROS core and connect to the UR5e, run the code on seperate terminal. If the connection is successful, you can see the simulation robot has the same pose as the real one. 
    ```
      roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100
    ```
    ```
      roslaunch ur5e_moveit_config moveit_planning_execution.launch
    ```
    ```
      roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz
    ```

  2. Start the program on the UR5e and you will see 'connected to reverse interface' on the first terminal running roscore, that means you can now control the robot with Moveit. You can interact with the end effector on Rviz window and press ```plan and execute``` to move the robot.

### Vision and remote control:
Remote control is achieved via ROS topic and Moveit python interface. Remote machine can run MATLAB with ROS toolbox to connect to the ROS machine. By running the MATLAB functions a ROS topic publisher is created as “control command” and sending commands to the ROS node. The ROS machine will create a ROS subscriber in python and listen to the command and execute the command with Moveit interface. Vision feedback is achieved in the same way but the publisher for vision is running on the ROS machine via python.
  - Run ```Computer_Vision.py``` to start corner detection and publishing the coordinates
  - Run ```Arm_control.py``` to set the simulation environment for trajectory planning, move the gripper to starting position and ready to recieve and execute control commands and CV feedback.

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
