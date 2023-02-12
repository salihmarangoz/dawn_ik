#include <salih_marangoz_thesis/robot_parser.h>

namespace salih_marangoz_thesis
{

RobotParser::RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : nh(nh), priv_nh(priv_nh)
{
  //test();
  parseCurrentRobot();
}

std::string RobotParser::createHeader()
{
  return "";
}

std::string RobotParser::createFooter()
{
  return "";
}

template <typename T>
std::string RobotParser::vector2Str(const std::string& variable, const std::vector<T>& arr)
{
  std::string out = "";
  out += variable;
  out += "[] = {";

  for (int i=0; i<arr.size(); i++)
  {
    out += std::to_string(arr[i]);

    if (i!=arr.size()-1)
    {
      out += ",";
    }
    else
    {
      out +="};";
    }
  }

  return out;
}


/// @brief Converts full transformations into translation arrays for code generation.
/// @param variable Variable name. For example: inline const TEST_VARIABLE
/// @param transformations Full transformations. Only translation part will be used.
/// @return Translation vectors in x,y,z order. Example output:
///         inline const TEST_VARIABLE[3][3] = {{0.500000, 0.200000, 0.300000},
///                                             {0.254654, 0.212310, 0.332100},
///                                             {0.456121, 0.546546, 0.100000}};
std::string RobotParser::eigenTranslation2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision)
{
  std::ostringstream out_stream;
  if (precision>=0)
  {
    out_stream.precision(precision);
  }

  std::string first_part = "";
  first_part += variable;
  first_part += "[";
  first_part += std::to_string(transformations.size());
  first_part += "]";
  first_part += "[3] = {";
  std::string spacing = std::string(first_part.size(), ' ');
  out_stream << first_part;

  for (int i=0; i<transformations.size(); i++)
  {
    auto translation = transformations[i].translation();
    if (i!=0) out_stream << spacing;
    out_stream << "{" << std::to_string(translation.x()) << ", " << std::to_string(translation.y()) << ", " << std::to_string(translation.z()) << "}";
    if (i!=transformations.size()-1) out_stream << ",";
    if (i==transformations.size()-1) out_stream << "};";
    out_stream << std::endl;
  }

  return out_stream.str();
}

/// @brief Converts full transformations into quaternion arrays for code generation.
/// @param variable Variable name. For example: inline const TEST_VARIABLE
/// @param transformations Full transformations. Only rotation part will be used.
/// @param precision Floating point precision. Uses default approach if the value is negative.
/// @return Quaternion vector in w,x,y,z order. Example output:
///         inline const TEST_VARIABLE[3][4] = {{0.891007, 0.000000, 0.000000, 0.453990},
///                                             {0.707107, 0.707107, 0.000000, 0.000000},
///                                             {0.154508, 0.024472, 0.975528, 0.154508}};
std::string RobotParser::eigenQuaternion2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision)
{
  std::ostringstream out_stream;
  if (precision>=0)
  {
    out_stream.precision(precision);
  }

  std::string first_part = "";
  first_part += variable;
  first_part += "[";
  first_part += std::to_string(transformations.size());
  first_part += "]";
  first_part += "[4] = {";
  std::string spacing = std::string(first_part.size(), ' ');
  out_stream << first_part;

  for (int i=0; i<transformations.size(); i++)
  {
    auto rotation = Eigen::Quaterniond(transformations[i].rotation());
    if (i!=0) out_stream << spacing;
    out_stream << "{" << std::to_string(rotation.w()) << ", " << std::to_string(rotation.x()) << ", " << std::to_string(rotation.y()) << ", " << std::to_string(rotation.z()) << "}";
    if (i!=transformations.size()-1) out_stream << ",";
    if (i==transformations.size()-1) out_stream << "};";
    out_stream << std::endl;
  }

  return out_stream.str();
}


// WARN: Assumed that joints are ordered for forward kinematic computations.
// WARN: Assumed that each joint can be single DOF or static. So, each joint can't have more than one variable.
// WARN: Assumed that all active joints rotate around Z axis.
// WARN: Assumed that collision objects are connected to the links that have a joint parent.
// WARN: Assumed that joint at 0 (zero) idx is the root joint (which has no parent link).
// WARN: Assumed that root joint has identity transformation to the child link.
// WARN: Assumed that acm is processed so that new collision pairs do not appear after adding the custom collision objects.
bool RobotParser::parseCurrentRobot()
{
  // Load robot model, robot state
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    return false;
  }
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  planning_scene_monitor->waitForCurrentRobotState(ros::Time(0));
  planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
  moveit::core::RobotState robot_state = lps->getCurrentState(); // copy the current state
  moveit::core::RobotModelConstPtr robot_model = robot_state.getRobotModel();

  const std::vector<const moveit::core::JointModel*> joint_models = robot_model->getJointModels();


  /////////////////////////////////////////////////////////////////////
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
  std::vector<bool> joint_is_position_bounded;
  std::vector<float> joint_max_position;
  std::vector<float> joint_min_position;
  std::vector<bool> joint_is_velocity_bounded;
  std::vector<float> joint_max_velocity;
  std::vector<float> joint_min_velocity;
  std::vector<bool> joint_is_acceleration_bounded;
  std::vector<float> joint_max_acceleration;
  std::vector<float> joint_min_acceleration;

  // Link info
  std::vector<std::string> link_names;
  std::vector<Eigen::Isometry3d> link_transform;
  std::vector<bool> link_can_skip_translation;
  std::vector<bool> link_can_skip_rotation;
  /////////////////////////////////////////////////////////////////////


  ROS_INFO("Available links in the robot model:");
  std::vector<std::string> all_links = robot_model->getLinkModelNames(); // WARN: Assuming that getLinkModelNames returns links in order with the indexes, starting from zero
  for (auto l: all_links)
  {
    const moveit::core::LinkModel* link_model = robot_state.getLinkModel(l);
    int link_index = link_model->getLinkIndex();
    ROS_INFO("Link name: %s, Link idx: %d", l.c_str(), link_index);
  }

  ROS_INFO("ACM:");
  collision_detection::AllowedCollisionMatrix acm = lps->getAllowedCollisionMatrix();
  acm.print(std::cout);
  ROS_INFO("Processed ACM:");
  Eigen::ArrayXXi processed_acm = Eigen::ArrayXXi::Zero(all_links.size(), all_links.size()); // values must be one or zero

  for (int i=0; i<all_links.size(); i++)
  {
    const moveit::core::LinkModel* link_model_1 = robot_state.getLinkModel(all_links[i]);
    for (int j=0; j<all_links.size(); j++)
    {
      const moveit::core::LinkModel* link_model_2 = robot_state.getLinkModel(all_links[j]);

      // Allow collision with itself
      if (i==j) 
      {
        processed_acm(i,j) = 1;
        continue;
      }

      // Skip checking the pairs for the upper half of the collision matrix
      if (j>i)
        continue;

      // Using the assumption, if one of these links has no collision, then there is no need to check for collision
      if (link_model_1->getShapes().size()==0 || link_model_2->getShapes().size()==0)
      {
        processed_acm(i,j) = 1;
        continue;
      }

      collision_detection::AllowedCollision::Type t;
      if (acm.getAllowedCollision(all_links[i], all_links[j], t))
      {
        if (t == collision_detection::AllowedCollision::CONDITIONAL) continue; // No support for conditional collisions
        processed_acm(i,j) = 1;
        continue;
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////
      // TODO: Extra rules for horti robot
      // Discard collision checking between other arms
      if (all_links[i].find("arm_") != std::string::npos && all_links[j].find("arm_") != std::string::npos)
      {
        ROS_WARN_ONCE("Applying extra rules for horti robot!!!");
        ROS_WARN("Discarding collision checking between %s and %s", all_links[i].c_str(), all_links[j].c_str());
        processed_acm(i,j) = 1;
        continue;
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
  }

  // Fill the upper half using the lower half
  for (int i=0; i<all_links.size(); i++)
    for (int j=0; j<all_links.size(); j++)
      if (j>i)
        processed_acm(i,j) = processed_acm(j,i);

  num_collision_pairs = processed_acm.size() - processed_acm.sum();
  ROS_INFO("Total number of collision pairs: %d", num_collision_pairs);
  std::cout << processed_acm << std::endl;
  

  // Number of joints will be more than variables in our case. But each joint can have its own transformation.
  for (auto j: joint_models)
  {
    ROS_INFO("------------------------------------------------------");
    ROS_INFO("Reading joint: %s", j->getName().c_str());

    int joint_idx = j->getJointIndex(); // joint idx in robot model
    ROS_INFO("joint idx: %d", joint_idx);

    // Get parent link
    const moveit::core::LinkModel* parent_link = j->getParentLinkModel(); // if j is the ROOT joint this will return NULL pointer!!!
    if (parent_link!=nullptr)
    {
      ROS_INFO("parent link name: %s", parent_link->getName().c_str());
    }
    else
    {
      ROS_INFO("no parent link available (current joint is the root joint!)");
    }

    // Get child link
    const moveit::core::LinkModel* child_link = j->getChildLinkModel(); // There will be always a link
    ROS_INFO("child link name: %s", child_link->getName().c_str());
    bool skip_translation = child_link->getJointOriginTransform().translation().isZero();
    bool skip_rotation = child_link->getJointOriginTransform().rotation().isIdentity();
    if (skip_translation)
      ROS_WARN("child link origin doesn't need translation!");
    if (skip_rotation)
    ROS_WARN("child link origin doesn't need rotation!");

    if (j->getVariableCount()==0)
    {
      ROS_INFO("Joint has no variables. Skipping...");
      continue;
    }
    if (j->getVariableCount()>1)
    {
      ROS_ERROR("Joint has too many variables. This is not supported!");
      continue;
    }

    // Get name and idxs
    std::string variable_name = j->getVariableNames()[0]; // For single DOF joints, this will be just the joint name
    int variable_idx = j->getFirstVariableIndex(); // variable idx in robot state
    ROS_INFO("variable name: %s, variable idx: %d", variable_name.c_str(), variable_idx);

    // Get variable limits
    moveit::core::JointModel::Bounds variable_bounds = j->getVariableBounds();
    bool is_position_bounded = variable_bounds[0].position_bounded_;
    float min_position = variable_bounds[0].min_position_;
    float max_position = variable_bounds[0].max_position_;
    bool is_velocity_bounded = variable_bounds[0].velocity_bounded_;
    float min_velocity = variable_bounds[0].min_velocity_;
    float max_velocity = variable_bounds[0].max_velocity_;
    bool is_acceleration_bounded = variable_bounds[0].acceleration_bounded_;
    float min_acceleration = variable_bounds[0].min_acceleration_;
    float max_acceleration = variable_bounds[0].max_acceleration_;

    if (is_position_bounded)
      ROS_INFO("min position: %f, max position: %f", min_position, max_position);
    else
      ROS_INFO("no position limits!");

    if (is_velocity_bounded)
      ROS_INFO("min velocity: %f, max velocity: %f", min_velocity, max_velocity);
    else
      ROS_INFO("no velocity limits!");

    if (is_acceleration_bounded)
      ROS_INFO("min acceleration: %f, max acceleration: %f", min_acceleration, max_acceleration);
    else
      ROS_INFO("no acceleration limits!");

    //(global_transform_from_parent -> apply joint_value from current state (maybe a parameter) -> apply child link transform)
    //(global transform links -> collisions)
    
  }


  /*
  for (int i=0; i<robot_state.getVariableCount(); i++)
  {
    
  }
  */
  



  /*
  // Follow the kinematic tree
  const moveit::core::JointModel* current_joint = robot_model->getRootJoint();
  ROS_INFO("Joint: %s", current_joint->getName().c_str());

  while (true)
  {
    // One child link is possible
    const moveit::core::LinkModel* current_link = current_joint->getChildLinkModel();
    ROS_INFO("Link: %s", current_link->getName().c_str());

    // Get link transform
    const Eigen::Isometry3d current_link_transform = current_link->getJointOriginTransform();
    if (!current_link_transform.translation().isZero())
    {
      ROS_INFO_STREAM("translation: " << current_link_transform.translation());
    }

    if (current_link_transform.rotation().isIdentity())
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " without joint rotation");
    }
    else
    {
      ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " with joint rotation");
    }

    const double* pos_ = robot_state.getJointPositions(current_joint->getName());
    float pos = *pos_;

    // Multiple joints are possible. But here we will pick the first one.
    const std::vector<const moveit::core::JointModel*> next_joints = current_link->getChildJointModels();
    if (next_joints.size() <= 0) break; // end of the kinematic tree
    current_joint = next_joints[0];
    ROS_INFO("Joint: %s", current_joint->getName().c_str());
  }
  */

  return true;
}


void RobotParser::test()
{
  std::vector<Eigen::Isometry3d> m_arr;
  m_arr.resize(3);
  m_arr[0] = Eigen::Isometry3d::Identity();
  m_arr[1] = Eigen::Isometry3d::Identity();
  m_arr[2] = Eigen::Isometry3d::Identity();
  m_arr[0].translate(Eigen::Vector3d(0.5, 0.2, 0.3));
  m_arr[0].rotate(Eigen::AngleAxisd(0.3*M_PI, Eigen::Vector3d::UnitZ()));
  m_arr[1].translate(Eigen::Vector3d(0.254654, 0.21231, 0.3321));
  m_arr[1].rotate(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()));
  m_arr[2].translate(Eigen::Vector3d(0.456121, 0.546546211, 0.1));
  m_arr[2].rotate(Eigen::AngleAxisd(0.1*M_PI, Eigen::Vector3d::UnitX()));
  m_arr[2].rotate(Eigen::AngleAxisd(0.9*M_PI, Eigen::Vector3d::UnitY()));

  std::cout << "Test eigenTranslation2Str: " << std::endl << eigenTranslation2Str("inline const TEST_VARIABLE", m_arr) << std::endl;

  std::cout << "Test eigenQuaternion2Str: " << std::endl << eigenQuaternion2Str("inline const TEST_VARIABLE", m_arr) << std::endl;
}



} // namespace salih_marangoz_thesis