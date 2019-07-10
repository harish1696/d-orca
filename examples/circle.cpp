#include "Simulator.hpp"
#include "Agent.hpp"

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "dorcacircle");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10.0);
  int agentNo = std::stoi(argv[1]);
  bool is_armed_ = false;
  bool update_goal = true;

  Simulator* sim = new Simulator(agentNo, argc, argv);

  // Setup RVO simulation.
  sim->setupSimulator();
  geometry_msgs::PoseStamped agent_pose = sim->getAgentPosition();
  agent_pose.pose.position.z = 5;
  sim->setAgentGoal(agent_pose);

  while(ros::ok()) {
    ros::Time start = ros::Time::now();
    ros::spinOnce();
    loop_rate.sleep();

    if (!(sim->hasAgentReachedGoal())) {
      //set agent's velocity to chosen velocity and update the agent's preferred velocity
      sim->updateAgent();

      // update the simulator and perform do step
      sim->updateSimulator();
    } else {
      // update the agent goal
      if (update_goal) {
        agent_pose = sim->getAgentPosition();
        agent_pose.pose.position.x = -agent_pose.pose.position.x;
        agent_pose.pose.position.y = -agent_pose.pose.position.y;
        sim->setAgentGoal(agent_pose);
        update_goal = false;
      }
    }
    std::cout << "Agent " << agentNo << " : " << ros::Time::now() - start << "\n";
  }

  sim->killSimulator();

  delete sim;

  return 0;
}
