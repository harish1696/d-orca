#ifndef AGENT_HPP
#define AGENT_HPP

#include "ros/ros.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "rvo2_3d/RVO.h"
#include "rvo2_3d/Definitions.h"

#include <string>

class Agent {
  private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub, vel_sub;
    ros::Publisher local_pos_pub, cmd_vel;
    ros::ServiceClient arming_client, set_mode_client;

    int agentNo, totalAgents;
    RVO::Vector3 agentGoal;
    mavros_msgs::State current_state;
    bool is_update_goal;

  public:
    Agent(int agentNo);
    ~Agent();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void velocity_cb(const geometry_msgs::Twist::ConstPtr& msg, int i);
    void publishInitialPose();
    RVO::Vector3 getPreferredVelocity();
    bool reachedGoal(double radius);
    void publishAgentVelocity(RVO::Vector3 vel);
    void arm();
    void setMode();
    void setCommand();
    void setAgentGoal(geometry_msgs::PoseStamped goal);
    void setAgentGoal();
    void setArmAndModeTopics();

    std::vector<geometry_msgs::PoseStamped> qpose;
    std::vector<geometry_msgs::Twist> qvel;
};

#endif
