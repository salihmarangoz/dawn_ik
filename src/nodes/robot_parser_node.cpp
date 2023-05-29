#include <dawn_ik/robot_parser.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_parser_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  ros::AsyncSpinner spinner(4);
  dawn_ik::RobotParser rp(nh, priv_nh);
  ros::waitForShutdown();
  return 0;
}

