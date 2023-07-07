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

More specifically, different platforms were used to control each section of the project. The gripper is controlled using MATLAB, the arm is controlled using python, and computer vision processing is done in C++. The high-level code which combines all three is written in python, allowing each section to communicate using ROS topics. An example of this inter-platform communication is displayed in Fgure (*PROVIDE LINK FOR PICTURE*).\\


###Gripper Control
Explaining how gripper control works


###Computer Vision Processing
Explaining how computer vision works


###Arm Control and Integration
Explaining how Arm Control and Integration works


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
