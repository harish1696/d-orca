#include "Subscriber.hpp"

Subscriber::Subscriber(int agentNo) {
  nh_.getParam("/dorca/totalAgents", totalAgents);
  nh_.getParam("/dorca/timeStep", timeStep);
  nh_.getParam("/dorca/neighborDist", neighborDist);
  nh_.getParam("/dorca/maxNeighbors", maxNeighbors);
  nh_.getParam("/dorca/timeHorizon", timeHorizon);
  nh_.getParam("/dorca/radius", radius);
  nh_.getParam("/dorca/maxSpeed", maxSpeed);

  this->agentNo = agentNo;

  agent = new Agent(agentNo);
  initialize();

  //gazebo_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Simulator::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh_.subscribe("/publisher/state", 0, &Simulator::publisherStateCallback, this, ros::TransportHints().tcpNoDelay());
  is_state_obtained_ = false;
  is_initialized_ = false;
  std::cout << "Simulator Setup\n";
}

Subscriber::~Subscriber() {}

void Subscriber::publisherStateCallback(const dorca::AgentStates& msg) {
  //std::cout << "Getting agent states from publisher.\n";
  if (is_initialized_) {
    for(int i = 0; i < msg.agent.size(); i++) {
      agent->qpose[i].pose.position.x = msg.agent[i].pose.pose.position.x;
      agent->qpose[i].pose.position.y = msg.agent[i].pose.pose.position.y;
      agent->qpose[i].pose.position.z = msg.agent[i].pose.pose.position.z;
      agent->qpose[i].pose.orientation.x = msg.agent[i].pose.pose.orientation.x;
      agent->qpose[i].pose.orientation.y = msg.agent[i].pose.pose.orientation.y;
      agent->qpose[i].pose.orientation.z = msg.agent[i].pose.pose.orientation.z;
      agent->qpose[i].pose.orientation.w = msg.agent[i].pose.pose.orientation.w;

      agent->qvel[i].linear.x = msg.agent[i].velocity.linear.x;
      agent->qvel[i].linear.y = msg.agent[i].velocity.linear.y;
      agent->qvel[i].linear.z = msg.agent[i].velocity.linear.z;

      agent->agent_mode[i] = msg.agent[i].mode;
      agent->agent_armed[i] = msg.agent[i].armed;
      is_state_obtained_ = true;
    }
  } else {
    initialize();
  }
}


void Subscriber::initialize() {
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
    agent->agent_mode.push_back("DEFAULT");
    agent->agent_armed.push_back(false);
  }

  is_initialized_ = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "message_subscriber");
  Subscriber *node = new Subscriber();

  ros::Rate *loop_rate = new ros::Rate(50);
  while (ros::ok()){
    loop_rate->sleep();
    ros::spinOnce();
  }
  return 0;
}
