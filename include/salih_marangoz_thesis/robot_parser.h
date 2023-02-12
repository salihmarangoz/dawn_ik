#ifndef __ROBOT_PARSER_H__
#define __ROBOT_PARSER_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <salih_marangoz_thesis/shapes.h>
#include <salih_marangoz_thesis/utils/utils.h>

// NOTES:
// - Often variables correspond to joint names as well (joints with one degree of freedom have one variable), 
//   but joints with multiple degrees of freedom have more variables.

namespace salih_marangoz_thesis
{

class RobotParser
{
public:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  collision_detection::AllowedCollisionMatrix acm;

  RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  template <typename T> std::string vector2Str(const std::string& variable, const std::vector<T>& arr);
  std::string eigenTranslation2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision=-1);
  std::string eigenQuaternion2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision=-1);
  std::string createHeader();
  std::string createFooter();
  bool parseCurrentRobot();

  void test();

};


} // namespace salih_marangoz_thesis


#endif // __ROBOT_PARSER_H__