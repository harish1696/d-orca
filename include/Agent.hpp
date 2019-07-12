/**
 * BSD 3-Clause LICENSE
 *
 * Copyright 2019 University of Maryland, College Park
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
    void setAgentGoal(geometry_msgs::PoseStamped goal);
    void setArmAndModeTopics();

    std::vector<geometry_msgs::PoseStamped> qpose;
    std::vector<geometry_msgs::Twist> qvel;
};

#endif
