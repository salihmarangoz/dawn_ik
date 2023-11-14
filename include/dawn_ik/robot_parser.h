#ifndef DAWN_IK_ROBOT_PARSER_H
#define DAWN_IK_ROBOT_PARSER_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <filesystem>
#include <iostream>
#include <cassert>
#include <sstream>
#include <cctype> // std::isdigit()

#include <dawn_ik/utils.h>
#include <dawn_ik/yaml.h>


namespace dawn_ik
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
  int num_objects;
  int num_acm_link_pairs; // number of zeros in the acm matrix
  // int num_targets; // GET FROM targets !!!
  int endpoint_link_idx; // FROM YAML

  // Mapping vectors
  std::vector<int> joint_idx_to_variable_idx; // -1 if no variable available. Can be used as joint_has_variable vector
  std::vector<int> variable_idx_to_joint_idx;
  std::vector<int> joint_idx_to_target_idx; // -1 if no target available.
  std::vector<int> target_idx_to_joint_idx;
  std::vector<int> object_idx_to_link_idx;

  // Joint info
  std::vector<std::string> joint_names;
  std::vector<int> joint_axis; // 0:undefined, 1:x, 2:y, 3:z, -1:-x, -2:-y, -3:-z
  std::vector<int> joint_child_link_idx;
  std::vector<int> joint_parent_link_idx; // -1 if no link available
  std::vector<int> joint_is_position_bounded; // bool
  std::vector<float> joint_preferred_position;
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
  
  // Collision object info
  std::vector<Eigen::Isometry3d> object_transform;
  std::vector<int> object_can_skip_translation;
  std::vector<int> object_can_skip_rotation;

  // Goal info
  bool enabled_preferred_joint_position_goal; // TODO
  std::vector<float> weight_preferred_joint_position_goal;
  //std::vector<float> goal_weight_joint_minimal_velocity; // TODO
  //std::vector<float> goal_weight_joint_minimal_acceleration; // TODO
  //std::vector<float> goal_weight_joint_minimal_jerk; // TODO


  /////////////////////////////////////////////////////////////////////

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;

  Yaml::Node cfg;

  RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  void test();
  bool parse();
  bool readCfg();
  std::string generateCodeForParsedRobot();
  bool saveCode(const std::string& code);

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

  std::vector<int> findPartialChain(const int endpoint_link);

  std::string removeNonNumericalChars(const std::string &str)
  {
      std::string result;
      for (char c : str) if (std::isdigit(c) || c == '.' || c == '-') result.push_back(c);
      return result;
  }

  template<typename T>
  std::vector<T> parseNumericArrayFromString(const std::string &str, const char separator=',')
  {
      std::vector<T> result;
      std::stringstream ss(str);
      std::string item;

      while (std::getline(ss, item, separator))
      {
          T value;
          std::istringstream iss(removeNonNumericalChars(item));
          iss >> value;
          result.push_back(value);
      }
      
      return result;
  }

};

} // namespace dawn_ik


#endif // DAWN_IK_ROBOT_PARSER_H