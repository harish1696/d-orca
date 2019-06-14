#include "Simulator.hpp"
#include "Agent.hpp"

using namespace std;

int main(int argc, char **argv) {
  // Initialize ROS node.
  ros::init(argc, argv, "dorcacircle");
  ros::NodeHandle nh;
  ros::Rate rate(30.0);
  int agentNo = std::stoi(argv[1]);

  //Agent* agent = new Agent();
  Simulator* sim = new Simulator(agentNo, argc, argv);
  
  ros::spinOnce();
  rate.sleep();

  // Setup RVO simulation.
  sim->setupSimulator();

  // Set rate for loop.
  ros::Rate loop_rate(10.0);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    sim->updateAgent();

    // update the simulator and perform do step
    sim->updateSimulator();
  }

  sim->killSimulator();

  delete sim;

  return 0;
}
