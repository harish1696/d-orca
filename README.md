## Introduction

The document describes the various dependencies and the associated installation process for using trialorcalib code package.
This package helps simulate multiple quadrotors in ROS/Gazebo platform with ORCA based collision avoidance.

--------------------------------------------------------------------------------------------------------------------------------------------------------------
## Dependencies
Ubuntu 16.04
ROS Kinetic
MAVROS
PX4
Gazebo 7.13
Multiple-sitl
--------------------------------------------------------------------------------------------------------------------------------------------------------------
## Installation

# ROS Kinetic, Gazebo 7.13, PX4 and MAVROS:
Please follow the installation as on http://dev.px4.io/en/setup/dev_env_linux.html#gazebo-with-ros

# Multiple-sitl:
To install multiple sitl, please follow the documentation on
https://github.com/acsl-mipt/multiple-sitl/tree/for-px4-pr

--------------------------------------------------------------------------------------------------------------------------------------------------------------
## Build trialorcalib
Follow the steps below to build trialorcalib package

Open Terminal and enter,
```
cd mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src
```
Here, extract and paste trialorcalib folder
```
cd ..
catkin_make
source devel/setup.bash
```
--------------------------------------------------------------------------------------------------------------------------------------------------------------
## Running the simulation

# Modifying the environment:
Paste the required .obj envionment in /Firmware/Tools/sitl_gazebo/models/
Modify the iris.world file in /Firmware/Tools/sitl_gazebo/worlds/ to include this envinronment model

# Running Gazebo:
Run multiple-sitl with the required number of quadrotors as described in https://github.com/acsl-mipt/multiple-sitl/tree/for-px4-pr
A Gazebo window opens with the quadrotors in the environment.

# Running trialorcalib:
Modify the number of quadrotors as follows,

Open terminal and enter,
```
cd ~/catkin_ws/src/trialorcalib/src
```
Here, open orcafinal.cpp file and modify noOfAgents (line 23) to the required number of quadrotors and save.

In the terminal type,
```
cd /rvo2_3d
```
Here, modify the model paths for the obj model of quadrotor and environment in RVOSimulator.cpp (line 167 and 214) and save.

In the terminal type,
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun trialorcalib orcafinal
```

Now, the quadrotors follow the waypoints (given in the text file path.txt) while avoiding collision between each other and the buildings.

press ctrl+c to exit
--------------------------------------------------------------------------------------------------------------------------------------------------------------
