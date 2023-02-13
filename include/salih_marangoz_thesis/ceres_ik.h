#ifndef __CERES_IK_H__
#define __CERES_IK_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace salih_marangoz_thesis
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class CeresIK
{
public:
  std::random_device rand_dev;
  std::mt19937 rand_gen;
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

/**
 * MinimalJointDisplacementGoal
*/
struct MinimalJointDisplacementGoal {
  MinimalJointDisplacementGoal(const double *init_target_positions)
  :init_target_positions(init_target_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets; i++)
    {
      residuals[i] = target_values[i] - init_target_positions[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double *init_target_positions)
   {
     return (new ceres::AutoDiffCostFunction<MinimalJointDisplacementGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new MinimalJointDisplacementGoal(init_target_positions)));
   }
   
   const double* init_target_positions;
};


/**
 * CenterJointsGoal
*/
struct CenterJointsGoal {
  CenterJointsGoal(const double* target_centers)
  : target_centers(target_centers){}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets; i++)
    {
      residuals[i] = target_values[i] - target_centers[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double* target_centers)
   {
     return (new ceres::AutoDiffCostFunction<CenterJointsGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new CenterJointsGoal(target_centers)));
   }
   
   const double* target_centers;
};


} // namespace salih_marangoz_thesis


#endif // __CERES_IK_H__