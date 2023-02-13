#ifndef __CERES_IK_H__
#define __CERES_IK_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


/////////////////////////////////////////////////////////////////
#include <salih_marangoz_thesis/robot_conf_single_xarm7.h>
/////////////////////////////////////////////////////////////////

namespace salih_marangoz_thesis
{

class CeresIK
{
public:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;


  CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  moveit::core::RobotState getCurrentRobotState();
  void loop();
  bool update(moveit::core::RobotState &current_state);

};


} // namespace salih_marangoz_thesis


#endif // __CERES_IK_H__