#include "publisher.hpp"

Publisher::Publisher() {
  nh_.getParam("/dorca/totalAgents", totalAgents);
  pub_state_ = nh_.advertise<dorca::AgentStates>("/publisher/state", 0);

  initialize();
  for (int i = 0; i < totalAgents; i++) {
     std::string s = "/uav" + std::to_string(i+1);
     state_sub = nh_.subscribe<mavros_msgs::State>(s+"/mavros/state", 10, boost::bind(&Publisher::state_cb, this, _1, i));
     vel_sub =  nh_.subscribe<geometry_msgs::Twist>(s+"/mavros/setpoint_velocity/cmd_vel_unstamped", 1, boost::bind(&Publisher::velocity_cb, this, _1, i));
     gazebo_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Publisher::gzmavposeCallback, this, ros::TransportHints().tcpNoDelay());
  }

  is_mode_obtained = false;
  is_pose_obtained_ = false;
  is_velocity_obtained = false;
}

void Publisher::state_cb(const mavros_msgs::State::ConstPtr& msg, int i) {
  agents.agent[i].mode = msg->mode;
  is_mode_obtained = true;
}

void Publisher::velocity_cb(const geometry_msgs::Twist::ConstPtr& msg, int i) {
  agents.agent[i].velocity.linear.x = msg->linear.x;
  agents.agent[i].velocity.linear.y = msg->linear.y;
  agents.agent[i].velocity.linear.z = msg->linear.z;
  is_velocity_obtained = true;
}

void Publisher::gzmavposeCallback(const gazebo_msgs::ModelStates& msg) {
  for(int i = 0; i < msg.pose.size(); i++) {
    std::string name = msg.name[i];
    std::string modelName = name.substr(0,4);
    if (agents.agent.size() == totalAgents) {
      // Currently works for 99 drones.
      if (name.length() < 8 && name.length() > 5 && modelName == std::string("iris")) {
        if (name.length() == 6) {
          std::string substring = name.substr(5,1);
          int index = stoi(substring, nullptr, 10);
          agents.agent[index-1].pose.pose.position.x = msg.pose[i].position.x;
          agents.agent[index-1].pose.pose.position.y = msg.pose[i].position.y;
          agents.agent[index-1].pose.pose.position.z = msg.pose[i].position.z;
          agents.agent[index-1].pose.pose.orientation.x = msg.pose[i].orientation.x;
          agents.agent[index-1].pose.pose.orientation.y = msg.pose[i].orientation.y;
          agents.agent[index-1].pose.pose.orientation.z = msg.pose[i].orientation.z;
          agents.agent[index-1].pose.pose.orientation.w = msg.pose[i].orientation.w;
        } else {
          std::string substring = name.substr(5,2);
          int index = stoi(substring, nullptr, 10);
          agents.agent[index-1].pose.pose.position.x = msg.pose[i].position.x;
          agents.agent[index-1].pose.pose.position.y = msg.pose[i].position.y;
          agents.agent[index-1].pose.pose.position.z = msg.pose[i].position.z;
          agents.agent[index-1].pose.pose.orientation.x = msg.pose[i].orientation.x;
          agents.agent[index-1].pose.pose.orientation.y = msg.pose[i].orientation.y;
          agents.agent[index-1].pose.pose.orientation.z = msg.pose[i].orientation.z;
          agents.agent[index-1].pose.pose.orientation.w = msg.pose[i].orientation.w;
        }
      }
      is_pose_obtained_ = true;
    } else {
      initialize();
    }
  }
}

void Publisher::initialize() {
  dorca::AgentState agent;
  for (int i = 0; i < totalAgents; i++) {
    agents.agent.push_back(agent);
  }
}

void Publisher::publishMessages() {
  if (is_mode_obtained && is_pose_obtained_ && is_velocity_obtained)
    pub_state_.publish(agents);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "message_publisher");
  Publisher *node = new Publisher();

  ros::Rate *loop_rate = new ros::Rate(50);
  while (ros::ok()){
    loop_rate->sleep();
    node->publishMessages();
    ros::spinOnce();
  }
  return 0;
}
