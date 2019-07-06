/**
 * BSD 3-Clause LICENSE
 *
 * Copyright <2019> <HARISH SAMPATHKUMAR>
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
 */

#ifndef AGENT_HPP
#define AGENT_HPP

/**
 *  @file    Agent.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief
 *
 *  @section DESCRIPTION
 *
 *  Class that is used to handle agent related queries from
 *
 */

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
    /**
    * @brief Creates an Agent object for the given agent id
    * @param agent ID
    */
    Agent(int agentNo);

    /**
    * @brief Destroys the Agent object
    */
    ~Agent();

    /**
    * @brief Callback function to listen to states
    * @param none
    * @return initial state of the robot after it respawns
    */
    void state_cb(const mavros_msgs::State::ConstPtr& msg);

    /**
    * @brief Resets the simulation each time robot crashes
    * @param none
    * @return initial state of the robot after it respawns
    */
    void velocity_cb(const geometry_msgs::Twist::ConstPtr& msg, int i);

    /**
    * @brief Publishes the initial pose of all agents
    * @param none
    * @return none
    */
    void publishInitialPose();

    /**
    * @brief Retrieves the preferred velocity of the agent
    * @param none
    * @return preferred velocity of the agent
    */
    RVO::Vector3 getPreferredVelocity();

    /**
    * @brief Checks if the agent reached its goal
    * @param radius around goal within which the agent is considered
    * to have reached its goal
    * @return true if agent reached its goal, false otherwise
    */
    bool reachedGoal(double radius);

    /**
    * @brief Publishes the velocity of the agent
    * @param velocity of the agent
    * @return none
    */
    void publishAgentVelocity(RVO::Vector3 vel);

    /**
    * @brief Arms the agent
    * @param none
    * @return none
    */
    void arm();

    /**
    * @brief Sets the agent goal
    * @param goal pose
    * @return none
    */
    void setAgentGoal(geometry_msgs::PoseStamped goal);

    /**
    * @brief Sets the service topics of the agent appropriately to arm it
    * @param none
    * @return none
    */
    void setArmAndModeTopics();

    std::vector<geometry_msgs::PoseStamped> qpose;
    std::vector<geometry_msgs::Twist> qvel;
    std::string agent_mode;
    bool is_agent_armed;
};

#endif
