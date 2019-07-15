# Distributed Optimal Reciprocal Collision Avoidance
Owing to the growth in the field of swarm robotics, simulation of multiple agents sharing a common workspace have gained increasing attention. This package provides one such platform with the support to spawn multiple agents(quadrotors) in a simulated environment(Gazebo). It also incorporates into it optimal reciprocal collision avoidance algorithm to help agents navigate the environment without colliding with each other. This package distinguishes itself from other similar packages by decentralizing the multi-agent simulation framework thus making the system scalable and increasing the point of failures. The package makes use of ROS message passing protocol for communication between agents and thus removes the need to manage all the agents centrally.

<center>
<figure class="video_container">
<iframe width="560" height="315" src="https://www.youtube.com/embed/LnALstxYSjM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</figure>
</center>

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
#### Documentation
Visit this [page](https://gamma.umd.edu/researchdirections/aerialswarm/dorca) for more info on installing and using the package.

#### Issues
Create issues when you identify bugs.

The package is maintained by

- [Harish Sampathkumar](harish1696@gmail.com)
- [Senthil Hariharan Arul](senthilhariharana@gmail.com)
- [Adarsh Jagan Sathyamoorthy](adarshjagan3895@gmail.com)
