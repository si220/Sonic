# Sonic - The robot hearing dog

## ELEC70015 Human Centred Robotics module at Imperial College London, March 2024

## Authors: 
Zack Reeves - UI  
Chun Hin Lau - Sound  
Arthika Sivathasan - Sound  
Petar Barakov - Sound  
Krrish Jain - Human Detection  
Gian-Luca Fenocchi - Vision / Hardware  
Donavon Clay - Hardware  
Lun Tan - Navigation  
Saifullah Ijaz - SLAM

## Description:

This repository contains all relevant files created used in the design process of Sonic, a Robotic Hearing Dog meant to assist people with hearing loss. The repository contains:

- The [ROS workspace](ros_ws) that includes all ROS packages required for the operation of the robot.
- The [3D models](3D-Prints) required to reconstruct the robotic arm, case for the LIDAR sensor, mounting bracket for the camera and the Jetson housing.
- The [User Interface Web App](ui) source code.
- The [figures](figures) used in the final report.

[mobileNet.py](ros_ws/src/human_detection/scripts/mobileNet.py) contains the code for human detection  
[hector_slam](ros_ws/src/hector_slam) contains the code for SLAM  
[nav.py](ros_ws/src/human_detection/scripts/nav.py) contains the code for robot navigation  
[sonic_arm](ros_ws/src/sonic_arm) contains the arduino code for the nudging mechanism  
[sound_pkg](ros_ws/src/sound_pkg) contains the code for the sound detection

Additionally, the repository contains a single Dockerfile and a single Makefile that are needed to create the virtual environment necessary for the operation of the robot.
