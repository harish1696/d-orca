## Introduction
The document describes the usage of d-orca package.

## Launching Gazebo with Multiple Iris drones
The scripts directory contains `multiUAV.sh` which can be used to create a custom roslaunch file depending on the number of iris quadrotors that has to be spawned.

For example, to create a roslaunch that can spawn 20 quadrotors on launch run the following command

```
./multiUAV.sh 20
```

Similarly the above command can be replaced with other numbers to create a roslaunch file that can launch as many quadrotors.

Before launching the roslaunch file it is necessary to source the px4 directory. To do that, go to Firmware directory and run the following commands to source it.

```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
```

Then go to the launch directory in Firmware and run the following command to launch gazebo with desired number of quadrotors in it.

```
roslaunch px4 multiUAVLaunch.launch
```
