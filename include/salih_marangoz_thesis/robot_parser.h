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

  RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);

};


} // namespace salih_marangoz_thesis


#endif // __ROBOT_PARSER_H__