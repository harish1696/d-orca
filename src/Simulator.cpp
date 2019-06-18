#include "Simulator.hpp"

Simulator::Simulator(int agentNo, int argc, char** argv) {
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
  InitGazebo(argc, argv);
  initialize();

  sub = node->Subscribe("/gazebo/default/pose/info", &Simulator::gazebocb, this);
  is_pose_obtained_ = false;
}

Simulator::~Simulator() {}

void Simulator::InitGazebo(int argc, char** argv) {
  gazebo::client::setup(argc, argv);
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
}

//Get current position feedback for all drones from Gazebo.
void Simulator::gazebocb(ConstPosesStampedPtr &_msg) {
  for (int i =0; i < _msg->pose_size(); ++i) {
    const ::gazebo::msgs::Pose &pose = _msg->pose(i);
    std::string name = pose.name();
    std::string modelName = name.substr(0,4);
    if (agent->qpose.size()) {
      // Currently works for 99 drones.
      if (name.length() < 8 && name.length() > 5 && modelName == std::string("iris")) {
        if (name.length() == 6) {
          std::string substring = name.substr(5,1);
          int index = stoi(substring, nullptr, 10);
          const ::gazebo::msgs::Vector3d &position = pose.position();
          agent->qpose[index-1].pose.position.x = position.x();
          agent->qpose[index-1].pose.position.y = position.y();
          agent->qpose[index-1].pose.position.z = position.z();
          agent->qpose[index-1].pose.orientation.x = pose.orientation().x();
          agent->qpose[index-1].pose.orientation.y = pose.orientation().y();
          agent->qpose[index-1].pose.orientation.z = pose.orientation().z();
          agent->qpose[index-1].pose.orientation.w = pose.orientation().w();
        } else {
          std::string substring = name.substr(5,2);
          int index = stoi(substring, nullptr, 10);
          const ::gazebo::msgs::Vector3d &position = pose.position();
          agent->qpose[index-1].pose.position.x = position.x();
          agent->qpose[index-1].pose.position.y = position.y();
          agent->qpose[index-1].pose.position.z = position.z();
          agent->qpose[index-1].pose.orientation.x = pose.orientation().x();
          agent->qpose[index-1].pose.orientation.y = pose.orientation().y();
          agent->qpose[index-1].pose.orientation.z = pose.orientation().z();
          agent->qpose[index-1].pose.orientation.w = pose.orientation().w();
        }
      }
      if (agent->qpose[agentNo].pose.position.x && agent->qpose[agentNo].pose.position.y)
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
  // Specify the global time step of the simulation.
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
  agent->setAgentGoal();
}

void Simulator::updateAgent() {
  if (agent->arm()) {
    // Check if agent reached goal
    agent->reachedGoal(rvo_sim_->getAgentRadius(agentNo));
    // Publish velocity to the agents.
    agent->publishAgentVelocity(rvo_sim_->getAgentVelocity(agentNo));
    // Set agent's preffered velocity (directed towards the target waypoint).
    rvo_sim_->setAgentPrefVelocity(agentNo, agent->getPreferredVelocity());
    // Set agent position in simulator
    rvo_sim_->setAgentPosition(agentNo, RVO::Vector3(agent->qpose[agentNo].pose.position.x, agent->qpose[agentNo].pose.position.y, agent->qpose[agentNo].pose.position.z));
  }
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
  sub.reset();
  node->Fini();

  gazebo::transport::fini();
  node.reset();
  gazebo::client::shutdown();

  delete rvo_sim_;
}
