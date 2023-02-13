#include <salih_marangoz_thesis/ceres_ik.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "salih_marangoz_thesis_ceres_ik");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  ros::AsyncSpinner spinner(4);
  salih_marangoz_thesis::CeresIK ik(nh, priv_nh);
  ros::waitForShutdown();
  return 0;
}