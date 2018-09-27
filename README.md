# Vision_Arm_Project
This is a 3D vision robotic arm demo based on Dobot Magician robotics


# File Description 
main.cpp  -- A C++ file that using OpenCV and Intel Real Sense Camera to detect object.

ransac_geometry.cpp  -- Ramsac algorithms help us to find plane, line and else.

test.py -- Demo file

python_vision -- Python API linked with main.cpp used in test.py

Dobot_Control -- Collect Data using API and send data to test.py using Soket

# How to run

In oder to run the whole project you have to install the below environment:

Python3 

Opencv3

Qt5

Cmake

Intel Realsense Library

Dobot Magician API

Socket

You also need to give USB access when connected to Dobot Magician. The project tested on Ubuntu 16.04.

# Demo

![alt text](https://github.com/rmhsawyer/Vision_Arm_Project/Demo1.JPG)

![alt text](https://raw.githubusercontent.com/rmhsawyer/Vision_Arm_Project.git/Demo1.JPG)

![alt text](https://raw.githubusercontent.com/rmhsawyer/Vision_Arm_Project/Demo1.JPG)

# Copyright 
Minghe Ren (sawyermh@bu.edu)
