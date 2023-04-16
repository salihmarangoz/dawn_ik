#ifndef __CERES_IK_H__
#define __CERES_IK_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h> // TODO

#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
#include <salih_marangoz_thesis/joint_trajectory_control_interface.h>
#include <salih_marangoz_thesis/utils.h>
#include <salih_marangoz_thesis/robot_monitor.h>
#include <salih_marangoz_thesis/goals.h>

namespace salih_marangoz_thesis
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class CeresIK
{
// my classes
public:
  std::shared_ptr<RobotMonitor> robot_monitor;
public:
  std::random_device rand_dev;
  std::mt19937 rand_gen;
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  std::shared_ptr<JointTrajectoryControlInterface> joint_controller;

  // TODO
  void subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg);
  ros::Subscriber endpoint_sub; 
  Eigen::Vector3d endpoint;
  bool endpoint_received = false;
  Eigen::Quaterniond direction;

  CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  moveit::core::RobotState getCurrentRobotState();
  void loop();
  bool update(moveit::core::RobotState &current_state);

};


} // namespace salih_marangoz_thesis


#endif // __CERES_IK_H__