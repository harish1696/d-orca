#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "rvo2_3d/RVO.h"

#include "ros/ros.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/ModelStates.h>
#include <vector>

#include "Agent.hpp"

class Simulator {
  private:
    ros::NodeHandle nh_;
    ros::Subscriber gazebo_pose_sub_;
    int totalAgents, agentNo, maxNeighbors;
    float timeStep, neighborDist, timeHorizon, radius, maxSpeed;
    bool is_pose_obtained_;
    RVO::RVOSimulator* rvo_sim_;
    Agent* agent;

  public:
    Simulator(int agentNo, int argc, char** argv);
    ~Simulator();
    void initialize();
    void setupSimulator();
    bool hasAgentReachedGoal();
    void updateSimulator();
    void updateAgent();
    void killSimulator();
    void gzmavposeCallback(const gazebo_msgs::ModelStates& msg);
    geometry_msgs::PoseStamped getAgentPosition();
    void setAgentGoal(geometry_msgs::PoseStamped goal);
};


#endif
