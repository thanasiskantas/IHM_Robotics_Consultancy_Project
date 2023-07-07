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

## Computer Vision

### Computer Vision part 1
- nick
- qiyang
```
ahsdhasidoashdas
```
## Implementation

To implement such a task, the team decided to integrate three components: The gripper, the arm, and computer vision. The gripper is responsible for the IHM, the arm is responsible for the pick-place task, and the computer vision works with both to achieve error detection and correction.

After all three components functioned correctly individually, they were integrated in a way like seen in Figure (*PROVIDE LINK FOR PICTURE*).

More specifically, different platforms were used to control each section of the project. The gripper is controlled using MATLAB, the arm is controlled using python, and computer vision processing is done in C++. The high-level code which combines all three is written in python, allowing each section to communicate using ROS topics. An example of this inter-platform communication is displayed in Fgure (*PROVIDE LINK FOR PICTURE*).\\


## Gripper Control
### Introduction
This algorithm is designed to utilise the finger to realise the in-hand manipulation for various sizes of objects, including sliding and rotation, with the goal of minimising the error. Now the algorithm is being tested on the following shapes: square cube, hexagon prism, octagon prism. 

### Simulation

In addition, the folder [Gripper Simulation](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/tree/bfcf90439357cf4e225c7dbe434d14aee228815c/Gripper%20Simulation) produces a simulation process carried out using Simulink to observe the motion of objects within the gripper. A spatial contact force block is introduced to simulate various friction forces. This simulation aided in understanding the behaviour of the system and allowed for the identification of preferred rotation start poses. 

With the help of simulation, six types of statuses within the gripper are observed for the cube with four of them being preferred rotate start pose. However, at the later stage of testing, the friction status of two fingers doesn’t show any impact on the pose of the cube, the pose is found to be purely dependent on the clockwise or anticlockwise rotation of the fingers. For example, during the slide, if the object moves from the left side to the right side with fingers rotated clockwise, regardless of the friction pad, the object ends at the pose shown in Figure 1.


### Rotation

A mathematical model for rotation is then presented, providing equations for clockwise and anticlockwise rotations.

$$R2L_{left motor} = π - cos^{-1}(\frac{d_L^2 + (W/2)^2 - (d/2)^2}{d_L * (W/2)}) - cos^{-1}(\frac{l_1^2 + 2 * L^2 - d_R^2}{4 * l_1 * L^2}) - cos^{-1}(\frac{d_L^2 + l_1^2 - d^2}{2 * d_L * l_1}) - θ_L $$

To compensate for the small angle in the undesired rotation prep-pose, an extra angle calculation is introduced, effectively removing the introduced error. Furthermore, a control strategy is designed to extend the limited rotation range of the gripper, allowing rotations from -180 to 180.

![Rotation Mathematical Model](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_2.png)

### Control Strategy
Another problem was the change in the thickness of the finger when changing the level of friction. This was solved by introducing a transitional layer is introduced between adjacent moves to ensure smooth transitions and precise open-loop control. Additionally, closed-loop control is implemented, incorporating a vision subsystem that provides real-time localization and feedback coordinates. More information regarding the vision subsystem can be found in the dedicated section of the report. 

![Control Strategy](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_3.png)

### Sliding

Regarding sliding, an important observation is that any two points within the gripper moving range could be reached with three circular arcs. Therefore, for sliding movements, a mathematical model consisting of three linked arcs is introduced, allowing the gripper to select the combination with the shortest arc length using gradient descent, thereby minimizing the error introduced by friction switches. The motor angles required for sliding movements are provided in the equations.

![Sliding Mathematical Model](https://github.com/thanasiskantas/IHM_Robotics_Consultancy_Project/blob/00b76ba407fd9bcee9a7d44300f1b258b95658ad/control_diagram_1.png)



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
