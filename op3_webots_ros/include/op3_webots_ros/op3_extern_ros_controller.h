#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <webots/Supervisor.hpp>

#include <yaml-cpp/yaml.h>

#define N_MOTORS (20)

namespace adol
{

class OP3ExternROSController : public webots::Supervisor 
{
public:
  OP3ExternROSController();
  virtual ~OP3ExternROSController();

  void run();

  void initialize(std::string gain_file_path);

  void process();

  void setDesiredJointAngles();
  void getPresentJointAngles();
  void getPresentJointTorques();
  
  void getCurrentRobotCOM();
  void getIMUOutput();

  void publishPresentJointStates();
  void publishIMUOutput();
  void publishCOMData();

  void myStep();
  void wait(int ms);

  int time_step_ms_;
  double time_step_sec_;

  double current_time_sec_; // control time

  // for motor angle
  double desired_joint_angle_rad_[N_MOTORS];
  double current_joint_angle_rad_[N_MOTORS];
  double current_joint_torque_Nm_[N_MOTORS];

  // center of mass
  geometry_msgs::Vector3 com_m_;
  double current_com_m_[3];
  double previous_com_m_[3];
  double current_com_vel_mps_[3];

  // imu
  double torso_xyz_m_[3];
  sensor_msgs::Imu imu_data_;

  // devices
  webots::LED* head_led_;
  webots::LED* body_led_;
  webots::Camera* camera_;
  webots::Speaker* speaker_;
  webots::Keyboard* key_board_; 
  
  webots::Gyro *gyro_;
  webots::Accelerometer *acc_;
  webots::InertialUnit *iu_;


  webots::Motor* motors_[N_MOTORS];
  webots::PositionSensor* encoders_[N_MOTORS];
  
  webots::Node *torso_node_;
  webots::Node *rf_node_, *lf_node_;

private:
  void queueThread();
  void posCommandCallback(const std_msgs::Float64::ConstPtr &msg, const int &joint_idx);

  bool parsePIDGainYAML(std::string gain_file_path);

  ros::Publisher present_joint_state_publisher_;
  ros::Publisher imu_data_publisher_;
  ros::Publisher com_data_publisher_;

  bool desired_joint_angle_rcv_flag_[20];

  sensor_msgs::JointState joint_state_msg_;

  boost::thread queue_thread_;
};

}