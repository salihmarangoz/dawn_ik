#include <salih_marangoz_thesis/robot_parser.h>

namespace salih_marangoz_thesis
{

RobotParser::RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : nh(nh), priv_nh(priv_nh)
{
  //test();
  parseCurrentRobot();
  std::cout << generateCodeForParsedRobot();
}

std::string RobotParser::strVector2Str(const std::string& variable, const std::vector<std::string>& arr)
{
  std::string out = "";
  out += variable;
  out += "[";
  out += std::to_string(arr.size());
  out += "] = {";

  for (int i=0; i<arr.size(); i++)
  {
    out += "\"";
    out += arr[i];
    out += "\"";

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
std::string RobotParser::eigenTranslation2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision, int extra_whitespace)
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
  std::string spacing = std::string(first_part.size()+extra_whitespace, ' ');
  out_stream << first_part;

  for (int i=0; i<transformations.size(); i++)
  {
    auto translation = transformations[i].translation();
    if (i!=0) out_stream << spacing;
    out_stream << "{" << std::to_string(translation.x()) << ", " << std::to_string(translation.y()) << ", " << std::to_string(translation.z()) << "}";
    if (i!=transformations.size()-1) out_stream << ",";
    if (i==transformations.size()-1) out_stream << "};";
    if (i!=transformations.size()-1) out_stream << std::endl;
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
std::string RobotParser::eigenQuaternion2Str(const std::string& variable, const std::vector<Eigen::Isometry3d>& transformations, int precision, int extra_whitespace)
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
  std::string spacing = std::string(first_part.size()+extra_whitespace, ' ');
  out_stream << first_part;

  for (int i=0; i<transformations.size(); i++)
  {
    auto rotation = Eigen::Quaterniond(transformations[i].rotation());
    if (i!=0) out_stream << spacing;
    out_stream << "{" << std::to_string(rotation.w()) << ", " << std::to_string(rotation.x()) << ", " << std::to_string(rotation.y()) << ", " << std::to_string(rotation.z()) << "}";
    if (i!=transformations.size()-1) out_stream << ",";
    if (i==transformations.size()-1) out_stream << "};";
    if (i!=transformations.size()-1) out_stream << std::endl;
  }

  return out_stream.str();
}

std::string RobotParser::eigenArrayXXi2Str(const std::string& variable, const Eigen::ArrayXXi& mat, int extra_whitespace)
{
  std::ostringstream out_stream;
  std::string first_part = "";
  first_part += variable;
  first_part += "[";
  first_part += std::to_string(mat.rows());
  first_part += "]";
  first_part += "[";
  first_part += std::to_string(mat.cols());
  first_part += "]";
  first_part += "= {";
  std::string spacing = std::string(first_part.size()+extra_whitespace, ' ');
  out_stream << first_part;

  for (int i=0; i<mat.rows(); i++)
  {
    if (i!=0) out_stream << spacing;
    out_stream << "{";
    for (int j=0; j<mat.cols(); j++)
    {
      //int val = mat(i,j);
      out_stream << std::to_string(mat(i,j));
      if (j==mat.cols()-1 && i!=mat.rows()-1) out_stream << "},";
      if (j==mat.cols()-1 && i==mat.rows()-1) out_stream << "}";
      if (j!=mat.cols()-1) out_stream << ",";
    }
    if (i==mat.rows()-1) out_stream << "};";
    if (i!=mat.rows()-1) out_stream << std::endl;
  }

  return out_stream.str();
}

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
  num_joints = joint_models.size();

  ROS_INFO("Available links in the robot model:");
  std::vector<std::string> all_links = robot_model->getLinkModelNames(); // WARN: Assuming that getLinkModelNames returns links in order with the indexes, starting from zero
  num_links = all_links.size();
  link_names.resize(num_links);
  link_transform.resize(num_links);
  link_can_skip_translation.resize(num_links);
  link_can_skip_rotation.resize(num_links);
  for (auto l: all_links)
  {
    const moveit::core::LinkModel* link_model = robot_state.getLinkModel(l);
    int link_index = link_model->getLinkIndex();
    ROS_INFO("Link name: %s, Link idx: %d", l.c_str(), link_index);

    link_names[link_index] = l.c_str();
    link_transform[link_index] = link_model->getJointOriginTransform();
    link_can_skip_translation[link_index] = link_model->getJointOriginTransform().translation().isZero();
    link_can_skip_rotation[link_index] = link_model->getJointOriginTransform().rotation().isIdentity();
  }

  ROS_INFO("ACM:");
  collision_detection::AllowedCollisionMatrix acm = lps->getAllowedCollisionMatrix();
  acm.print(std::cout);
  ROS_INFO("Processed ACM:");
  processed_acm = Eigen::ArrayXXi::Zero(all_links.size(), all_links.size()); // values must be one or zero

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
        ROS_FATAL_ONCE("Applying extra rules for horti robot!!!");
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


  ROS_INFO("Available joints in the robot model:");
  // Number of joints will be more than variables in our case. But each joint can have its own transformation.
  num_variables = 0;
  joint_idx_to_variable_idx.resize(num_joints, -1);
  variable_idx_to_joint_idx.resize(num_joints); // will be shrink after this block
  joint_names.resize(num_joints);
  joint_child_link_idx.resize(num_joints);
  joint_parent_link_idx.resize(num_joints);
  joint_is_position_bounded.resize(num_joints);
  joint_max_position.resize(num_joints);
  joint_min_position.resize(num_joints);
  joint_is_velocity_bounded.resize(num_joints);
  joint_max_velocity.resize(num_joints);
  joint_min_velocity.resize(num_joints);
  joint_is_acceleration_bounded.resize(num_joints);
  joint_max_acceleration.resize(num_joints);
  joint_min_acceleration.resize(num_joints);
  for (auto j: joint_models)
  {
    ROS_INFO("------------------------------------------------------");
    ROS_INFO("Reading joint: %s", j->getName().c_str());

    int joint_idx = j->getJointIndex(); // joint idx in robot model
    ROS_INFO("joint idx: %d", joint_idx);
    joint_names[joint_idx] = j->getName();

    // Get parent link
    const moveit::core::LinkModel* parent_link = j->getParentLinkModel(); // if j is the ROOT joint this will return NULL pointer!!!
    if (parent_link!=nullptr)
    {
      ROS_INFO("parent link name: %s", parent_link->getName().c_str());
      joint_parent_link_idx[joint_idx] = parent_link->getLinkIndex();
    }
    else
    {
      ROS_INFO("no parent link available (current joint is the root joint!)");
      joint_parent_link_idx[joint_idx] = -1;
    }

    // Get child link
    const moveit::core::LinkModel* child_link = j->getChildLinkModel(); // There will be always a link
    ROS_INFO("child link name: %s", child_link->getName().c_str());
    joint_child_link_idx[joint_idx] = child_link->getLinkIndex();

    bool skip_translation = child_link->getJointOriginTransform().translation().isZero();
    bool skip_rotation = child_link->getJointOriginTransform().rotation().isIdentity();
    if (skip_translation)
      ROS_WARN("child link origin doesn't need translation!");
    if (skip_rotation)
    ROS_WARN("child link origin doesn't need rotation!");

    // Check if the current joint has a supported variable
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
    num_variables++;

    // Get variable name and idxs
    std::string variable_name = j->getVariableNames()[0]; // For single DOF joints, this will be just the joint name
    int variable_idx = j->getFirstVariableIndex(); // variable idx in robot state
    ROS_INFO("variable name: %s, variable idx: %d", variable_name.c_str(), variable_idx);
    joint_idx_to_variable_idx[joint_idx] = variable_idx;
    variable_idx_to_joint_idx[variable_idx] = joint_idx;

    // Get variable limits
    moveit::core::JointModel::Bounds variable_bounds = j->getVariableBounds();
    joint_is_position_bounded[joint_idx] = variable_bounds[0].position_bounded_;
    joint_min_position[joint_idx] = variable_bounds[0].min_position_;
    joint_max_position[joint_idx] = variable_bounds[0].max_position_;
    joint_is_velocity_bounded[joint_idx] = variable_bounds[0].velocity_bounded_;
    joint_min_velocity[joint_idx] = variable_bounds[0].min_velocity_;
    joint_max_velocity[joint_idx] = variable_bounds[0].max_velocity_;
    joint_is_acceleration_bounded[joint_idx] = variable_bounds[0].acceleration_bounded_;
    joint_min_acceleration[joint_idx] = variable_bounds[0].min_acceleration_;
    joint_max_acceleration[joint_idx] = variable_bounds[0].max_acceleration_;

    if (joint_is_position_bounded[joint_idx])
      ROS_INFO("min position: %f, max position: %f", joint_min_position[joint_idx], joint_max_position[joint_idx]);
    else
      ROS_INFO("no position limits!");

    if (joint_is_velocity_bounded[joint_idx])
      ROS_INFO("min velocity: %f, max velocity: %f", joint_min_velocity[joint_idx], joint_max_velocity[joint_idx]);
    else
      ROS_INFO("no velocity limits!");

    if (joint_is_acceleration_bounded[joint_idx])
      ROS_INFO("min acceleration: %f, max acceleration: %f", joint_min_acceleration[joint_idx], joint_max_acceleration[joint_idx]);
    else
      ROS_INFO("no acceleration limits!");
  }
  variable_idx_to_joint_idx.resize(num_variables);

  return true;
}

std::string RobotParser::generateCodeForParsedRobot()
{
  std::string prefix = "const ";
  std::string header_guard_name = "__AUTOGENERATED_ROBOT_CONFIGURATION__";
  std::string namespace_name = "robot";

  std::ostringstream out_stream;

  // header guard start
  out_stream << "#ifndef " << header_guard_name << std::endl;
  out_stream << "#define " << header_guard_name << std::endl;
  out_stream << std::endl;

  // libs
  out_stream << "#include <vector>" << std::endl;
  out_stream << "#include <string>" << std::endl;
  out_stream << "#include <Eigen/Dense>" << std::endl;
  out_stream << std::endl;

  // namespace start
  out_stream << "namespace " << namespace_name << std::endl;
  out_stream << "{" << std::endl;
  out_stream << std::endl;

  // Constants
  out_stream << "// Constants" << std::endl;
  out_stream << prefix << "int num_joints = " << num_joints << ";" << std::endl;
  out_stream << prefix << "int num_variables = " << num_variables << ";" << std::endl;
  out_stream << prefix << "int num_links = " << num_links << ";" << std::endl;
  out_stream << prefix << "int num_collision_pairs = " << num_collision_pairs << ";" << std::endl;
  out_stream << std::endl;

  // Mapping vectors
  out_stream << "// Mapping vectors" << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_idx_to_variable_idx", joint_idx_to_variable_idx) << " // -1 if no variable available. Can be used as joint_has_variable vector" << std::endl;
  out_stream << prefix << primitiveVector2Str("int variable_idx_to_joint_idx", variable_idx_to_joint_idx) << std::endl;
  out_stream << std::endl;

  // Joint info
  out_stream << "// Joint info" << std::endl;
  out_stream << prefix << strVector2Str("std::string joint_names", joint_names) << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_child_link_idx", joint_child_link_idx) << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_parent_link_idx", joint_parent_link_idx) << " // -1 if no link available" << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_is_position_bounded", joint_is_position_bounded) << " // bool" << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_max_position", joint_max_position) << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_min_position", joint_min_position) << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_is_velocity_bounded", joint_is_velocity_bounded) << " // bool" << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_max_velocity", joint_max_velocity) << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_min_velocity", joint_min_velocity) << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_is_acceleration_bounded", joint_is_acceleration_bounded) << " // bool" << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_max_acceleration", joint_max_acceleration) << std::endl;
  out_stream << prefix << primitiveVector2Str("double joint_min_acceleration", joint_min_acceleration) << std::endl;
  out_stream << std::endl;

  // Link info
  out_stream << "// Link info" << std::endl;
  out_stream << prefix << strVector2Str("std::string link_names", link_names) << std::endl;
  out_stream << prefix << eigenTranslation2Str("double link_transform_translation_only", link_transform, 10, prefix.size()) << std::endl;
  out_stream << prefix << eigenQuaternion2Str("double link_transform_quaternion_only", link_transform, 10, prefix.size()) << std::endl;
  out_stream << prefix << primitiveVector2Str("int link_can_skip_translation", link_can_skip_translation) << " // bool" << std::endl;
  out_stream << prefix << primitiveVector2Str("int link_can_skip_rotation", link_can_skip_rotation) << " // bool" << std::endl;
  out_stream << std::endl;

  // ACM
  out_stream << "// ACM" << std::endl;
  out_stream << prefix << eigenArrayXXi2Str("int processed_acm", processed_acm, prefix.size());
  out_stream << std::endl;

  // namespace end
  out_stream << "} // namespace " << namespace_name << std::endl;
  out_stream << std::endl;

  // header guardend
  out_stream << "#endif // " << header_guard_name << std::endl;
  return out_stream.str();
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