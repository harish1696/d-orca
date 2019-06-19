# Decentralized Optimal Reciprocal Collision Avoidance
Owing to the growth in the field of swarm robotics, simulation of multiple agents sharing a common workspace have gained increasing attention. This package provides one such platform with the support to spawn multiple agents(quadrotors) in a simulated environment(Gazebo). It also incorporates into it optimal reciprocal collision avoidance algorithm to help agents navigate the environment without colliding with each other. This package distinguishes itself from other similar packages by decentralizing the multi-agent simulation framework thus making the system scalable and increasing the point of failures. The package makes use of ROS message passing protocol for communication between agents and thus removes the need to manage all the agents centrally.

# Installing the d-orca package
Before using the package, create a catkin workspace using the following commands

```
mkdir -p ~/dorca_ws
cd ~/dorca_ws
mkdir -p src
catkin_make
```

and run the following command in the /src directory of the workspace to clone the d-orca package

```
git clone https://github.com/harish1696/d-orca
cd /d-orca
git submodule update --init
```

Before building the d-orca package,  the following dependencies have to be installed

## Dependencies
Ubuntu 16.04
ROS Kinetic
MAVROS
PX4
Gazebo 7.13

## Installation
__Note__ :The following dependencies are for Ubuntu 16.04

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

And then build your catkin workspace

```
cd ~/dorca_ws
catkin_make
```
