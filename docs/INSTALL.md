## Introduction
The document describes the various dependencies and the associated installation process for using d-orca package.

## Dependencies
Ubuntu 16.04
ROS Kinetic
MAVROS
PX4
Gazebo 7.13
Multiple-sitl

## Installation
__Note__ : Make sure to use Ubuntu 16.04

# ROS Kinetic, Gazebo 7.13 and MAVROS:
Run the `install_dependency.sh` script available in the scripts folder.

# PX4 firmware
Go to the Firware directory in the d-orca package and run the following commands
```
make px4_sitl_default gazebo
```

Once the firmware is built run the following command to source the environment
```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```
