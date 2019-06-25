#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "rvo2_3d/RVO.h"

#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include <dorca/AgentState.h>
#include <dorca/AgentStates.h>
#include <vector>

class Publisher {
  private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub, vel_sub, gazebo_pose_sub_;
    ros::Publisher pub_state_;
    int agentNo, totalAgents;
    dorca::AgentStates agents;
    bool is_mode_obtained, is_pose_obtained_, is_velocity_obtained;

  public:
    Publisher();
    ~Publisher();
    void initialize();
    void gzmavposeCallback(const gazebo_msgs::ModelStates& msg);
    void publisherStateCallback(const dorca::AgentStates& msg);
};

#endif
