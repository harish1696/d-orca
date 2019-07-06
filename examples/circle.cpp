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

#include "Simulator.hpp"
#include "Agent.hpp"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "dorcacircle");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);
  int agentNo = std::stoi(argv[1]);
  bool is_armed_ = false;

  Simulator* sim = new Simulator(agentNo);

  // Setup RVO simulation.
  sim->setupSimulator();
  geometry_msgs::PoseStamped agent_pose = sim->getAgentPosition();
  agent_pose.pose.position.z = 5;
  sim->setAgentGoal(agent_pose);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    if (!(sim->hasAgentReachedGoal())) {
      //set agent's velocity to chosen velocity and update the agent's preferred velocity
      sim->updateAgent();

      // update the simulator and perform do step
      sim->updateSimulator();
    } else {
      // update the agent goal
      agent_pose = sim->getAgentPosition();
      agent_pose.pose.position.x = -agent_pose.pose.position.x;
      agent_pose.pose.position.y = -agent_pose.pose.position.y;
      sim->setAgentGoal(agent_pose);
    }
  }

  sim->killSimulator();

  delete sim;

  return 0;
}
