#ifndef __ROBOT_PARSER_H__
#define __ROBOT_PARSER_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <salih_marangoz_thesis/utils.h>
#include <salih_marangoz_thesis/yaml.h>

namespace salih_marangoz_thesis
{

// NOTE:
// Two types of computation blocks:
//(global_transform_from_parent -> apply joint_value from current state (maybe a parameter) -> apply child link transform)
//(global transform links -> collisions)

// WARN: Assumed that joints are ordered for forward kinematic computations.
// WARN: Assumed that each joint can be single DOF or static. So, each joint can't have more than one variable.
// WARN: Assumed that all active joints rotate around Z axis.
// WARN: Assumed that collision objects are connected to the links that have a joint parent.
// WARN: Assumed that joint at 0 (zero) idx is the root joint (which has no parent link).
// WARN: Assumed that root joint has identity transformation to the child link.
// WARN: Assumed that acm is processed so that new collision pairs should not appear after adding the custom collision objects.
class RobotParser
{
public:
  /////////////////////////////////////////////////////////////////////
  // PARSED ROBOT INFO
  /////////////////////////////////////////////////////////////////////
  // Constants
  int num_joints;
  int num_variables;
  int num_links;
  int num_collision_pairs;

  // Mapping vectors
  std::vector<int> joint_idx_to_variable_idx; // -1 if no variable available. Can be used as joint_has_variable vector
  std::vector<int> variable_idx_to_joint_idx;

  // Joint info
  std::vector<std::string> joint_names;
  std::vector<int> joint_child_link_idx;
  std::vector<int> joint_parent_link_idx; // -1 if no link available
  std::vector<int> joint_is_position_bounded; // bool
  std::vector<float> joint_max_position;
  std::vector<float> joint_min_position;
  std::vector<int> joint_is_velocity_bounded; // bool
  std::vector<float> joint_max_velocity;
  std::vector<float> joint_min_velocity;
  std::vector<int> joint_is_acceleration_bounded; // bool
  std::vector<float> joint_max_acceleration;
  std::vector<float> joint_min_acceleration;

  // Link info
  std::vector<std::string> link_names;
  std::vector<int> link_parent_joint_idx;
  std::vector<Eigen::Isometry3d> link_transform; 
  std::vector<int> link_can_skip_translation;  // bool
  std::vector<int> link_can_skip_rotation;  // bool

  // ACM
  Eigen::ArrayXXi processed_acm;
  /////////////////////////////////////////////////////////////////////

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;

  RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  void test();
  bool parse();
  std::string generateCodeForParsedRobot();

  // String utilities
  std::string eigenTranslation2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision=-1, int extra_whitespace=0);
  std::string eigenQuaternion2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision=-1, int extra_whitespace=0);
  std::string eigenArrayXXi2Str(const std::string& variable, const Eigen::ArrayXXi& mat, int extra_whitespace);
  std::string strVector2Str(const std::string& variable, const std::vector<std::string>& arr);

  template <typename T>
  std::string primitiveVector2Str(const std::string& variable, const std::vector<T>& arr)
  {
    std::string out = "";
    out += variable;
    out += "[";
    out += std::to_string(arr.size());
    out += "] = {";
    for (int i=0; i<arr.size(); i++)
    {
      out += std::to_string(arr[i]);
      if (i!=arr.size()-1)
        out += ",";
      else
        out +="};";
    }
    return out;
  }

};


} // namespace salih_marangoz_thesis


#endif // __ROBOT_PARSER_H__