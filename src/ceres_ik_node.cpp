#include <salih_marangoz_thesis/ceres_ik.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "salih_marangoz_thesis_ceres_ik");
  salih_marangoz_thesis::CeresIK ik();
  return 0;
}