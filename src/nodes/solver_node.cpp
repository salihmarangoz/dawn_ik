#include <dawn_ik/dawn_ik.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "solver_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  ros::AsyncSpinner spinner(0);
  dawn_ik::DawnIK ik(nh, priv_nh);
  ros::waitForShutdown();
  return 0;
}