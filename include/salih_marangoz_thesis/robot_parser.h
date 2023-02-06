#ifndef __ROBOT_PARSER_H__
#define __ROBOT_PARSER_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <salih_marangoz_thesis/shapes.h>
#include <salih_marangoz_thesis/utils/utils.h>

namespace salih_marangoz_thesis
{

class RobotParser
{
public:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  moveit::core::RobotModelConstPtr kinematic_model;


  std::vector<std::string> link_names;
  std::vector<int> link_model_index;
  std::vector<Eigen::Isometry3d> link_transform;
  std::vector<bool> link_transform_translation_needs_computation;
  std::vector<bool> link_transform_rotation_needs_computation;

  std::vector<std::string> joint_names;
  std::vector<int> joint_variable_index;
  std::vector<float> joint_value_upper_limit;
  std::vector<float> joint_value_lower_limit;
  std::vector<bool> joint_transform_needs_computation; // decided using the active/passive state according to the move group


  RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  template <typename T> std::string vector2Str(std::string variable, const std::vector<T>& arr);
  std::string createHeader();
  std::string createFooter();

};


} // namespace salih_marangoz_thesis


#endif // __ROBOT_PARSER_H__