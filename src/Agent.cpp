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

#include "Agent.hpp"

Agent::Agent(int agentNo) {
  this->agentNo = agentNo;
  nh.getParam("/dorca/totalAgents", totalAgents);
  std::string s = "/uav" + std::to_string(agentNo + 1);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(s+"/mavros/setpoint_position/local", 10);
  cmd_vel = nh.advertise<geometry_msgs::Twist>(s+"/mavros/setpoint_velocity/cmd_vel_unstamped", 0);
  state_sub = nh.subscribe<mavros_msgs::State>(s+"/mavros/state", 10, &Agent::state_cb, this);

  for (int i = 0; i < totalAgents; i++) {
     s = "/uav" + std::to_string(i+1);
     if (i != agentNo)
     vel_sub =  nh.subscribe<geometry_msgs::Twist>(s+"/mavros/setpoint_velocity/cmd_vel_unstamped", 1, boost::bind(&Agent::velocity_cb, this, _1, i));
  }
  publishInitialPose();
  is_update_goal = true;
}

Agent::~Agent() {}

void Agent::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

void Agent::velocity_cb(const geometry_msgs::Twist::ConstPtr& msg, int i) {
  qvel[i].linear.x = msg->linear.x;
  qvel[i].linear.y = msg->linear.y;
  qvel[i].linear.z = msg->linear.z;
}

void Agent::setArmAndModeTopics() {
  std::string s = "/uav" + std::to_string(agentNo + 1);
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(s+"/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(s+"/mavros/set_mode");
}

void Agent::publishInitialPose() {
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(pose);
  }
}

void Agent::setAgentGoal(geometry_msgs::PoseStamped goal) {
  agentGoal = RVO::Vector3(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  std::cout << "Agent " << agentNo << " Goal set to: " << agentGoal << "\n";
}

void Agent::publishAgentVelocity(RVO::Vector3 vel) {
  geometry_msgs::Twist velo;
  velo.linear.x = vel.x();
  velo.linear.y = vel.y();
  velo.linear.z = vel.z();
  cmd_vel.publish(velo);
}

// Set current preffered velocity for the agents/obstacles.
RVO::Vector3 Agent::getPreferredVelocity() {
  RVO::Vector3 goalVector = agentGoal;
  RVO::Vector3 posVector = RVO::Vector3(qpose[agentNo].pose.position.x, qpose[agentNo].pose.position.y, qpose[agentNo].pose.position.z);
  goalVector = goalVector - posVector;

  if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
  }

  return goalVector;
}

bool Agent::reachedGoal(double radius) {
  // Construct the current position vector of the agent.
  RVO::Vector3 posVector = RVO::Vector3(qpose[agentNo].pose.position.x, qpose[agentNo].pose.position.y, qpose[agentNo].pose.position.z);

  // Check if agent reached its goal.
  if (RVO::absSq(posVector - agentGoal) < radius * radius) {
    return true;
  }
  return false;
}

void Agent::arm() {
  if (current_state.mode != "OFFBOARD") {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client.call(offb_set_mode);
  } else {
    if (!current_state.armed) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      arming_client.call(arm_cmd);
    }
  }
}
