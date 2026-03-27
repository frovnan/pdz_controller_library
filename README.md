# controller_library

## Description

A ROS 2 Humble package that will someday contain various works by students from Pd|Z for the Franka Emika FR3 robot arm. 

These include:
* [Cartesian Impedance Controller](https://github.com/LucasG2001/cartesian_impedance_control)
* [Cartesian Admittance Controller](https://github.com/LucasG2001/admittance_control)
* Joint Impedance Controller
* [Hybrid Force/Impedance Contoller](﻿https://github.com/krombier/Bachelor_Thesis_Force_Control)
* [Hybrid Impedance/Admittance Controller](https://github.com/enjoyericbu/hybridcontroller)
* [Potential Field Method](https://github.com/anel-b/repulsive_force)
* [Riemann Motion Policy](https://github.com/acaviezel/riemannian_motion_policy)
* [Singularity and Oscillation Avoidance](https://github.com/GeniusT31/src)

## Prerequisites

* Ubuntu 22.04 LTS+ (Linux)
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [franka_ros2 v0.13.1](https://support.franka.de/docs/franka_ros2.html)

## Download

Change your current directory in the terminal to **/franka_ros2_ws/src**. Clone the repository into this folder. Then, navigate back to the **/franka_ros2_ws** directory, build the package with colcon, and source the setup file.

```bash
cd franka_ros2_ws/src
git clone https://github.com/frovnan/BA_thesis_controller_library.git
cd franka_ros2_ws
colcon build --packages-select BA_thesis_controller_library
source install/setup.bash
```

In order to not have to source the setup file after every build, you can add the following line at the end of your .bashrc file (which you can access by executing `nano .bashrc` in your home directory): 

```bash
source /home/<user>/franka_ros2_ws/install/setup.bash
```

(to be continued...)
