#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/Float64Stamped.h>
#include <webots_ros/node_get_center_of_mass.h>

#include <string>

#define TIME_STEP_MS (8)

std::string op3_joint_names[20] = {
    "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
    "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
    "head_pan", "head_tilt"};

std::string webots_joint_names[20] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, 
    "PelvYR" /*ID7 */, "PelvYL" /*ID8 */, "PelvR" /*ID9 */, "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, 
    "AnkleR" /*ID15*/, "AnkleL" /*ID16*/, "FootR" /*ID17*/, "FootL" /*ID18*/, 
    "Neck" /*ID19*/, "Head" /*ID20*/
};

double goal_joint_angles_rad[20] = {0};
double present_joint_angles_rad[20];
double present_joint_torques_Nm_[20];

bool goal_joint_angle_rcv_flag[20];

webots_ros::set_int time_step_srv;

ros::Publisher present_joint_state_publisher;

ros::Subscriber goal_pos_subs[20];
ros::Subscriber present_pos_subs[20];
ros::Subscriber present_torque_subs[20];

ros::ServiceClient webots_time_step;

ros::ServiceClient set_pos_clients[20];
ros::ServiceClient pos_sensor_enable_clients[20];
ros::ServiceClient torque_feedback_enable_clients_[20];

ros::ServiceClient supervisor_node_client_;

ros::ServiceClient get_com_client_;

sensor_msgs::JointState joint_state_msg_;

webots_ros::node_get_center_of_mass com_srv_;

ros::Publisher present_com_publisher_;

uint64_t supervisor_node_ = 0;;

void posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx);
void presentJointAnglesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx);
void presentJointTorquesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx);

void initializePositionSensors();
void initializeTorqueFeedback();

void sendPresentPosition();
void setGoalPosition();
void getCOMofRobot();

void process();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_webots_node");
  ros::NodeHandle nh;
  
  // Wait for the Webots and ROS controller
  ros::service::waitForService("/robot/time_step");

  /* Publishers, Subsribers, and Service Clients */
  for(int i = 0; i < 20; i++)
  {
    // make subscribers for the joint position topic from robotis framework
    std::string goal_pos_topic_name = "/robotis_op3/" + op3_joint_names[i] + "_position/command";
    goal_pos_subs[i] = nh.subscribe<std_msgs::Float64>(goal_pos_topic_name, 1, boost::bind(posCommandCallback, _1, i));
    
    // make service clients for setting goal position in webots
    std::string set_pos_srv_name = "/" + webots_joint_names[i] + "/set_position";  
    set_pos_clients[i] = nh.serviceClient<webots_ros::set_float>(set_pos_srv_name);
    
    // make service clients for enabling joint position sensors in webots
    std::string pos_sensor_enable_srv_name = "/" + webots_joint_names[i] + "S/enable";  
    pos_sensor_enable_clients[i] = nh.serviceClient<webots_ros::set_int>(pos_sensor_enable_srv_name);

    // make service clients for enabling joint torque feedback in webots
    std::string torque_feedback_enable_srv_name = "/" + webots_joint_names[i] + "/torque_feedback_sensor/enable";  
    torque_feedback_enable_clients_[i] = nh.serviceClient<webots_ros::set_int>(torque_feedback_enable_srv_name);

    // make subscribers for getting present pos angle rad from webots
    std::string pos_sensor_topic_name = "/" + webots_joint_names[i] + "S/value";  
    present_pos_subs[i] = nh.subscribe<webots_ros::Float64Stamped>(pos_sensor_topic_name, 1, boost::bind(presentJointAnglesCallback, _1, i));

    // make subscribers for getting present torque Nm from webots
    std::string torque_feedback_topic_name = "/" + webots_joint_names[i] + "/torque_feedback";  
    present_torque_subs[i] = nh.subscribe<webots_ros::Float64Stamped>(torque_feedback_topic_name, 1, boost::bind(presentJointTorquesCallback, _1, i));
  }
  
  // make present joint state publisher
  present_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/robotis_op3/joint_states", 1);

  // make service client for time step
  webots_time_step = nh.serviceClient<webots_ros::set_int>("/robot/time_step");
  time_step_srv.request.value = TIME_STEP_MS;

  // get supervisor node
  supervisor_node_client_ = nh.serviceClient<webots_ros::get_uint64>("/supervisor/get_self");
  webots_ros::get_uint64 supervisor_node_srv;
  supervisor_node_srv.request.ask = true;
  supervisor_node_client_.call(supervisor_node_srv);
  supervisor_node_ = supervisor_node_srv.response.value;

  // make service client for com
  get_com_client_ = nh.serviceClient<webots_ros::node_get_center_of_mass>("/supervisor/node/get_center_of_mass");

  // make a com publisher
  present_com_publisher_ = nh.advertise<geometry_msgs::Point>("/robotis_op3/present_center_of_mass", 1);

  usleep(1000*1000);
  // Initialize Webots
  initializePositionSensors();
  initializeTorqueFeedback();
  webots_time_step.call(time_step_srv);
  usleep(8*1000);
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  sendPresentPosition();
  ros::spinOnce();
  
  ros::Rate rate(125);

  while(ros::ok())
  {
    getCOMofRobot();
    setGoalPosition();
    sendPresentPosition();

    webots_time_step.call(time_step_srv);
    ros::spinOnce();
    //process();
    rate.sleep();
  }
  return 0;
}

void process()
{
  bool goal_pos_flag = true;

  // while (ros::ok())
  // {
  //   for (int i = 0; i < 20; i++)
  //   {
  //     goal_pos_flag = goal_pos_flag & goal_joint_angle_rcv_flag[i];
  //   }

  //   if (goal_pos_flag)
  //     break;
    
  //   goal_pos_flag = true;
  //   ros::spinOnce();
  // }
  getCOMofRobot();
  setGoalPosition();
  sendPresentPosition();
}

void initializePositionSensors()
{
  webots_ros::set_int srv;
  srv.request.value = TIME_STEP_MS;
  for(int i = 0; i < 20; i++)
  {
    pos_sensor_enable_clients[i].call(srv);
    if (srv.response.success == 0)
      ROS_ERROR_STREAM("Failed to enable the position sensor of " << webots_joint_names[i]);
  }
}

void initializeTorqueFeedback()
{
  webots_ros::set_int srv;
  srv.request.value = TIME_STEP_MS;
  for(int i = 0; i < 20; i++)
  {
    torque_feedback_enable_clients_[i].call(srv);
    if (srv.response.success == 0)
      ROS_ERROR_STREAM("Failed to enable the torque feedback of " << webots_joint_names[i]);
  }
}

void posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx)
{
  goal_joint_angles_rad[joint_idx] = msg->data;
  goal_joint_angle_rcv_flag[joint_idx] = true;
}

void presentJointAnglesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx)
{
  present_joint_angles_rad[joint_idx] = msg->data;
}

void presentJointTorquesCallback(const webots_ros::Float64Stamped::ConstPtr &msg, const int &joint_idx)
{
  present_joint_torques_Nm_[joint_idx] = msg->data;
}

void sendPresentPosition()
{
  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  joint_state_msg_.header.stamp = ros::Time::now();
    
  for(int i = 0; i < 20; i++)
  {
    joint_state_msg_.name.push_back(op3_joint_names[i]);
    joint_state_msg_.position.push_back(present_joint_angles_rad[i]);
    joint_state_msg_.velocity.push_back(0);
    joint_state_msg_.effort.push_back(present_joint_torques_Nm_[i]);
  }

  present_joint_state_publisher.publish(joint_state_msg_);
}

void setGoalPosition()
{
  webots_ros::set_float srv;
  for(int i = 0; i < 20; i++)
  {
    goal_joint_angle_rcv_flag[i] = false;
    srv.request.value = goal_joint_angles_rad[i];
    set_pos_clients[i].call(srv);

    if (srv.response.success == 0)
      ROS_ERROR_STREAM("Failed to set the goal position of " << webots_joint_names[i]);
  }
}

void getCOMofRobot()
{
  com_srv_.request.node = supervisor_node_;
  get_com_client_.call(com_srv_);

  present_com_publisher_.publish(com_srv_.response.centerOfMass);
}