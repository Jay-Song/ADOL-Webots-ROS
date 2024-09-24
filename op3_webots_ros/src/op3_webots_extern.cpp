#include <ros/ros.h>

#include "op3_webots_ros/op3_extern_ros_controller.h"

std::string gain_file_path;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_webots_extern");
  ros::NodeHandle nh;
  nh.param<std::string>("gain_file_path", gain_file_path, "");

  adol::OP3ExternROSController *controller = new adol::OP3ExternROSController();

  controller->initialize(gain_file_path);

  ros::Duration(0.3).sleep();

  ros::Rate rate(125);

  while(ros::ok())
  {
    controller->process();
    ros::spinOnce();
    //process();
    //rate.sleep();
  }
 
  delete controller;

  return EXIT_FAILURE;
}