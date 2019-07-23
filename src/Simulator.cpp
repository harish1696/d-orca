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

#include "Simulator.hpp"

Simulator::Simulator(int agentNo) {
  nh_.getParam("/dorca/totalAgents", totalAgents);
  nh_.getParam("/dorca/timeStep", timeStep);
  nh_.getParam("/dorca/neighborDist", neighborDist);
  nh_.getParam("/dorca/maxNeighbors", maxNeighbors);
  nh_.getParam("/dorca/timeHorizon", timeHorizon);
  nh_.getParam("/dorca/radius", radius);
  nh_.getParam("/dorca/maxSpeed", maxSpeed);

  this->agentNo = agentNo;

  rvo_sim_ = new RVO::RVOSimulator();
  agent = new Agent(agentNo);
  initialize();

  //sub = node->Subscribe("/gazebo/default/pose/info", &Simulator::gazebocb, this);
  gazebo_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Simulator::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());
  is_pose_obtained_ = false;
}

Simulator::~Simulator() {}

void Simulator::gzmavposeCallback(const gazebo_msgs::ModelStates& msg) {
  for(int i = 0; i < msg.pose.size(); i++) {
    std::string name = msg.name[i];
    std::string modelName = name.substr(0,4);
    if (agent->qpose.size()) {
      // Currently works for 99 drones.
      if (name.length() < 8 && name.length() > 5 && modelName == std::string("iris")) {
        if (name.length() == 6) {
          std::string substring = name.substr(5,1);
          int index = stoi(substring, nullptr, 10);
          agent->qpose[index-1].pose.position.x = msg.pose[i].position.x;
          agent->qpose[index-1].pose.position.y = msg.pose[i].position.y;
          agent->qpose[index-1].pose.position.z = msg.pose[i].position.z;
          agent->qpose[index-1].pose.orientation.x = msg.pose[i].orientation.x;
          agent->qpose[index-1].pose.orientation.y = msg.pose[i].orientation.y;
          agent->qpose[index-1].pose.orientation.z = msg.pose[i].orientation.z;
          agent->qpose[index-1].pose.orientation.w = msg.pose[i].orientation.w;
        } else {
          std::string substring = name.substr(5,2);
          int index = stoi(substring, nullptr, 10);
          agent->qpose[index-1].pose.position.x = msg.pose[i].position.x;
          agent->qpose[index-1].pose.position.y = msg.pose[i].position.y;
          agent->qpose[index-1].pose.position.z = msg.pose[i].position.z;
          agent->qpose[index-1].pose.orientation.x = msg.pose[i].orientation.x;
          agent->qpose[index-1].pose.orientation.y = msg.pose[i].orientation.y;
          agent->qpose[index-1].pose.orientation.z = msg.pose[i].orientation.z;
          agent->qpose[index-1].pose.orientation.w = msg.pose[i].orientation.w;
        }
      }
      is_pose_obtained_ = true;
    } else {
      initialize();
    }
  }
}

void Simulator::initialize() {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = 0;
  pose.pose.orientation.z = 0;

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;
  for (int i = 0; i < totalAgents; i++) {
    agent->qpose.push_back(pose);
    agent->qvel.push_back(vel);
  }
}

void Simulator::setupSimulator() {
  rvo_sim_->setTimeStep(timeStep);

  // Specify the default parameters for agents that are subsequently added.
  rvo_sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, radius, maxSpeed);

  // Add agents/obstacle, specifying their start position, and store their inital waypoint in the environment.
  for (int i = 0; i < totalAgents; i ++) {
    rvo_sim_->addAgent(RVO::Vector3(agent->qpose[i].pose.position.x, agent->qpose[i].pose.position.y, agent->qpose[i].pose.position.z));
  }

  agent->setArmAndModeTopics();
  while (!is_pose_obtained_) {
    ros::spinOnce();
  }
}

geometry_msgs::PoseStamped Simulator::getAgentPosition() {
  return agent->qpose[agentNo];
}

void Simulator::setAgentGoal(geometry_msgs::PoseStamped goal) {
  agent->setAgentGoal(goal);
}

bool Simulator::hasAgentReachedGoal() {
  // Check if agent reached goal
  return agent->reachedGoal(rvo_sim_->getAgentRadius(agentNo));
}

void Simulator::updateAgent() {
  // arm the agent
  agent->arm();
  // Publish velocity to the agents.
  agent->publishAgentVelocity(rvo_sim_->getAgentVelocity(agentNo));
  // Set agent's preffered velocity (directed towards the target waypoint).
  rvo_sim_->setAgentPrefVelocity(agentNo, agent->getPreferredVelocity());
  // Set agent position in simulator
  rvo_sim_->setAgentPosition(agentNo, RVO::Vector3(agent->qpose[agentNo].pose.position.x, agent->qpose[agentNo].pose.position.y, agent->qpose[agentNo].pose.position.z));
}

void Simulator::updateSimulator() {
  for (int i = 0; i < totalAgents; i ++) {
    RVO::Vector3 posVector = RVO::Vector3(agent->qpose[i].pose.position.x, agent->qpose[i].pose.position.y, agent->qpose[i].pose.position.z);
    rvo_sim_->setAgentPosition(i,posVector);
    if (i != agentNo) {
      rvo_sim_->setAgentVelocity(i, RVO::Vector3(agent->qvel[i].linear.x, agent->qvel[i].linear.y, agent->qvel[i].linear.z));
    }
  }
  rvo_sim_->doStep();
}

void Simulator::killSimulator() {
  delete rvo_sim_;
}
