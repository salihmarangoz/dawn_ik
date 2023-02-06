#include <salih_marangoz_thesis/robot_parser.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "salih_marangoz_thesis_robot_parser");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  ros::AsyncSpinner spinner(4);
  salih_marangoz_thesis::RobotParser rp(nh, priv_nh);
  ros::waitForShutdown();
  return 0;
}

