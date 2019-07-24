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
    float timeStep, neighborDist, timeHorizon, radius, reachedGoalRadius, maxSpeed;
    bool is_pose_obtained_;
    RVO::RVOSimulator* rvo_sim_;
    Agent* agent;

  public:
    Simulator(int agentNo);
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
