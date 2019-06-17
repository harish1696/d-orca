# Introduction

Before using the package, create a catkin workspace using the following commands

```
mkdir -p ~/catkin_ws
cd ~/catkin_ws
mkdir -p src
catkin_make
```

and run the following command in the /src directory of the workspace to clone the d-orca package

```
git clone https://github.com/harish1696/d-orca
git submodule update --init
```

And then build your catkin workspace

```
cd ~/catkin_ws
catkin_make
```
