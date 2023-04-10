#include <salih_marangoz_thesis/robot_parser.h>

// EXAMPLE YAML USAGE:
// Yaml::Node& joint_position_limits_override = cfg["joint_position_limits_override"];
// for(auto it = joint_position_limits_override.Begin(); it != joint_position_limits_override.End(); it++)
// {
//   ROS_WARN("%s", (*it).second["joint_name"].As<std::string>().c_str() );
//   if ( ! (*it).second["min_position"].IsNone() )
//     ROS_WARN("min_position %f", (*it).second["min_position"].As<double>() );
//   if ( ! (*it).second["max_position"].IsNone() )
//     ROS_WARN("max_position %f", (*it).second["max_position"].As<double>() );
//   if ( ! (*it).second["no_limit"].IsNone() )
//     ROS_WARN("no_limit %d", (*it).second["no_limit"].As<bool>() );
// }

namespace salih_marangoz_thesis
{

RobotParser::RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : nh(nh), priv_nh(priv_nh)
{
  // Read robot configuration from the given yaml file
  if (!readCfg())
  {
    ROS_FATAL("Reading yaml failed! Terminating...");
    exit(-1);
  }

  // Parse robot model using MoveIt and modify accordingly with the yaml file
  if (!parse())
  {
    ROS_FATAL("Parsing failed! Terminating...");
    exit(-1);
  }

  //std::cout << generateCodeForParsedRobot();

  // Save the generated code inside the package
  if (!saveCode(generateCodeForParsedRobot()))
  {
    ROS_FATAL("Saving the generated file failed! Terminating...");
    exit(-1);
  }

  test();

  ROS_INFO("Finished succesfully!");
  exit(0);
}

bool
RobotParser::readCfg()
{
  // Load robot configuration yaml file
  std::string cfg_filename;
  if (!priv_nh.getParam("cfg", cfg_filename))
  {
    ROS_FATAL("Configuration file not found!");
    return false;
  }
  std::string cfg_filepath = ros::package::getPath("salih_marangoz_thesis") + "/cfg/" + cfg_filename + ".yaml";
  ROS_WARN("Loading cfg: %s", cfg_filepath.c_str());
  try
  {
      Yaml::Parse(cfg, cfg_filepath.c_str()); // Use with .c_str(). See: https://github.com/jimmiebergmann/mini-yaml/issues/10
  }
  catch (const Yaml::Exception e)
  {
      std::cout << "Exception " << e.Type() << ": " << e.what() << std::endl;
      return false;
  }
  return true;
}

bool
RobotParser::saveCode(const std::string& code)
{
  std::string robot_configuration_folder = ros::package::getPath("salih_marangoz_thesis") + "/include/salih_marangoz_thesis/robot_configuration"; // TODO: hardcoded
  std::string generated_code_destination = robot_configuration_folder + "/autogen_test.h"; // TODO: hardcoded

  ROS_INFO("Generated code will be saved to: %s", generated_code_destination.c_str());
  if (std::filesystem::exists(generated_code_destination))
  {
    // TODO: check if the code needs an update. if not, then describe it and exit by returning true

    int i=0;
    std::string backup_path = generated_code_destination + ".old." + std::to_string(i);
    while(std::filesystem::exists(backup_path))
    {
      i++;
      backup_path = generated_code_destination + ".old." + std::to_string(i);
    }
    std::filesystem::rename(generated_code_destination, backup_path);
    ROS_WARN("Found a previously generated code. Saved a backup: %s", backup_path.c_str());
  }

  std::ofstream myfile (generated_code_destination);
  if (myfile.is_open())
  {
    myfile << code;
    myfile.close();
  }
  else
  {
    ROS_FATAL("Couldn't save the generated file. File: %s", generated_code_destination.c_str());
    return false;
  } 

  return true;
}

bool RobotParser::parse()
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
  if (num_joints == 0) return false;

  auto endpoint_link = robot_state.getLinkModel(cfg["endpoint_link"].As<std::string>());
  if (endpoint_link == nullptr)
  {
    ROS_FATAL("endpoint_link is not available!");
    return false;
  }
  endpoint_link_idx = endpoint_link->getLinkIndex();
  if (endpoint_link_idx<=0)
  {
    ROS_FATAL("endpoint_link_idx is %d. Something looks wrong!", endpoint_link_idx);
    return false;
  }

  ROS_INFO("Available links in the robot model:");
  std::vector<std::string> all_links = robot_model->getLinkModelNames(); // WARN: Assuming that getLinkModelNames returns links in order with the indexes, starting from zero
  num_links = all_links.size();
  link_names.resize(num_links);
  link_parent_joint_idx.resize(num_links);
  link_transform.resize(num_links);
  link_can_skip_translation.resize(num_links);
  link_can_skip_rotation.resize(num_links);
  for (auto l: all_links)
  {
    const moveit::core::LinkModel* link_model = robot_state.getLinkModel(l);
    int link_index = link_model->getLinkIndex();
    ROS_INFO("Link name: %s, Link idx: %d", l.c_str(), link_index);

    link_names[link_index] = l.c_str();
    link_parent_joint_idx[link_index] = link_model->getParentJointModel()->getJointIndex();
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
      // Extra rules for horti robot
      // Discard collision checking between other arms
      if (cfg["extra"]["horti_acm_tricks"].As<bool>())
      {
        if (all_links[i].find("arm_") != std::string::npos && all_links[j].find("arm_") != std::string::npos)
        {
          ROS_FATAL_ONCE("Applying extra rules for horti robot!!!");
          ROS_WARN("Discarding collision checking between %s and %s", all_links[i].c_str(), all_links[j].c_str());
          processed_acm(i,j) = 1;
          continue;
        }
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

  // Find partial chain to get targets, exclude targets according to yaml, exclude static targets
  auto tmp_targets = findPartialChain(endpoint_link_idx);
  for (int i=0; i<tmp_targets.size(); i++)
  {
    int curr_joint_idx = tmp_targets[i];
    bool curr_discard = false;

    // Exclude joints according to the given yaml
    if (cfg["exclude_joints"].IsSequence())
    {
      for(auto it_cfg = cfg["exclude_joints"].Begin(); it_cfg != cfg["exclude_joints"].End(); it_cfg++)
      {
        std::string excluded_joint_name = (*it_cfg).second.As<std::string>();
        auto excluded_joint_model = robot_state.getJointModel(excluded_joint_name);
        if (excluded_joint_model != nullptr && excluded_joint_model->getJointIndex() == curr_joint_idx)
        {
          ROS_WARN("Discarded joint %s according to the YAML configuration", joint_names[curr_joint_idx].c_str());
          curr_discard = true;
          break;
        }
      }
    }

    // Exclude joints if no variable is available (e.g. static joints)
    if (joint_idx_to_variable_idx[curr_joint_idx] < 0)
    {
      ROS_WARN("Discarded joint idx %s because it has no variable to control.", joint_names[curr_joint_idx].c_str());
      curr_discard = true;
    }

    if (!curr_discard)
      target_idx_to_joint_idx.push_back(curr_joint_idx);
  }
  // Also include the reverse mapping
  joint_idx_to_target_idx.resize(num_joints, -1);
  for (int i=0; i<target_idx_to_joint_idx.size(); i++)
  {
    int joint_idx = target_idx_to_joint_idx[i];
    joint_idx_to_target_idx[joint_idx] = i;
  }

  // Debug targets
  ROS_WARN("Joint optimization targets from root to endpoint link are:");
  for (int i=0; i<target_idx_to_joint_idx.size(); i++)
  {
    int joint_idx = target_idx_to_joint_idx[i];
    ROS_INFO("Joint: %s", joint_names[joint_idx].c_str());
  }
  ROS_WARN("---- end ----");

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
  out_stream << "#include <ceres/ceres.h>" << std::endl;
  out_stream << "#include <hpp/fcl/shape/geometric_shapes.h>" << std::endl;
  out_stream << std::endl;

  // using namespace
  out_stream << "using namespace ceres;" << std::endl;
  out_stream << std::endl;

  // supported shapes
  out_stream << "using hpp::fcl::CollisionObject;" << std::endl;
  out_stream << "// supported collision shapes:" << std::endl;
  out_stream << "using hpp::fcl::Box;" << std::endl;
  out_stream << "using hpp::fcl::Sphere;" << std::endl;
  out_stream << "using hpp::fcl::Capsule;" << std::endl;
  out_stream << "using hpp::fcl::Cylinder;" << std::endl;
  out_stream << "using hpp::fcl::Plane;" << std::endl;
  out_stream << std::endl;

  // namespace start
  out_stream << "namespace " << namespace_name << std::endl;
  out_stream << "{" << std::endl;
  out_stream << std::endl;

  // Constants
  out_stream << "// Constants" << std::endl;
  out_stream << prefix << "int endpoint_link_idx = " << endpoint_link_idx << ";" << std::endl;
  out_stream << prefix << "int num_joints = " << num_joints << ";" << std::endl;
  out_stream << prefix << "int num_variables = " << num_variables << ";" << std::endl;
  out_stream << prefix << "int num_links = " << num_links << ";" << std::endl;
  out_stream << prefix << "int num_collision_pairs = " << num_collision_pairs << ";" << std::endl;
  out_stream << prefix << "int num_targets = " << target_idx_to_joint_idx.size() << ";" << std::endl;
  out_stream << std::endl;

  // Mapping vectors
  out_stream << "// Mapping vectors" << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_idx_to_variable_idx", joint_idx_to_variable_idx) << " // -1 if no variable available. Can be used as joint_has_variable vector" << std::endl;
  out_stream << prefix << primitiveVector2Str("int variable_idx_to_joint_idx", variable_idx_to_joint_idx) << std::endl;
  out_stream << prefix << primitiveVector2Str("int joint_idx_to_target_idx", joint_idx_to_target_idx) << std::endl;
  out_stream << prefix << primitiveVector2Str("int target_idx_to_joint_idx", target_idx_to_joint_idx) << std::endl;
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
  out_stream << prefix << primitiveVector2Str("int link_parent_joint_idx", link_parent_joint_idx) << std::endl;
  out_stream << prefix << eigenTranslation2Str("double link_transform_translation_only", link_transform, 10, prefix.size()) << std::endl;
  out_stream << prefix << eigenQuaternion2Str("double link_transform_quaternion_only", link_transform, 10, prefix.size()) << std::endl;
  out_stream << prefix << primitiveVector2Str("int link_can_skip_translation", link_can_skip_translation) << " // bool" << std::endl;
  out_stream << prefix << primitiveVector2Str("int link_can_skip_rotation", link_can_skip_rotation) << " // bool" << std::endl;
  out_stream << std::endl;

  // ACM
  out_stream << "// ACM" << std::endl;
  out_stream << prefix << eigenArrayXXi2Str("int processed_acm", processed_acm, prefix.size());
  out_stream << std::endl;
  out_stream << std::endl;

  // Solver Options
  out_stream << "// Solver Options" << std::endl;
  out_stream << "static inline void setSolverOptions(ceres::Solver::Options &options)" << std::endl;
  out_stream << "{" << std::endl;
  if (cfg["solver_options"].IsMap())
  {
    for(auto it_cfg = cfg["solver_options"].Begin(); it_cfg != cfg["solver_options"].End(); it_cfg++)
    {
      std::string key = (*it_cfg).first;
      std::string value = (*it_cfg).second.As<std::string>();
      out_stream << "  options." << key << " = " << value << ";" << std::endl;
    }
  }
  else
  {
    ROS_FATAL("There was a problem while parsing solver_options !!!");
  }
  out_stream << "}" << std::endl;
  out_stream << std::endl;

  out_stream << "// Collision Objects" << std::endl;
  out_stream << "template<typename T>\n"
                "CollisionObject* inflatedCollisionObject(const T &shape, double inflation)\n"
                "{\n"
                "  return new CollisionObject(std::make_shared<T>(shape.inflated(inflation).first));\n"
                "}\n" << std::endl;
  out_stream << "static inline std::vector<CollisionObject*> getRobotCollisionObjects()" << std::endl;
  out_stream << "{" << std::endl;
  out_stream << "  const int inflation = " << cfg["proximity"]["inflation"].As<double>() << ";" << std::endl;
  out_stream << "  std::vector<CollisionObject*> objects;" << std::endl;
  out_stream << std::endl;
  if (cfg["proximity"]["objects"].IsSequence())
  {
    out_stream << "  objects.reserve(" << cfg["proximity"]["objects"].Size() << ")" << std::endl;
    for(auto it_cfg = cfg["proximity"]["objects"].Begin(); it_cfg != cfg["proximity"]["objects"].End(); it_cfg++)
    {
      auto item = (*it_cfg).second;
      out_stream << "  objects.push_back( inflatedCollisionObject(" << item["shape"].As<std::string>() << ", inflation) );" << std::endl;
    }
  }
  out_stream << std::endl;
  out_stream << "  return objects;" << std::endl;
  out_stream << "}" << std::endl;
  out_stream << std::endl;

  // Problem Constraints
  out_stream << "inline static void setProblemConstraints(ceres::Problem &problem, double *targets_ptr, double *targets_init)" << std::endl;
  out_stream << "{" << std::endl;
  // // joint1
  // problem.SetParameterLowerBound(targets, 0, -0.5);
  // problem.SetParameterUpperBound(targets, 0, 0.5);

  // // joint2
  // problem.SetParameterLowerBound(targets, 1, -0.5);
  // problem.SetParameterUpperBound(targets, 1, 0.5);

  // // joint3
  // problem.SetParameterLowerBound(targets, 2, -0.5);
  // problem.SetParameterUpperBound(targets, 2, 0.5);
  out_stream << "}" << std::endl;
  out_stream << std::endl;

  // namespace end
  out_stream << "} // namespace " << namespace_name << std::endl;
  out_stream << std::endl;

  // header guardend
  out_stream << "#endif // " << header_guard_name << std::endl;
  return out_stream.str();
}

































////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// UTILS //////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotParser::test()
{
  ROS_WARN("====================================== TEST ======================================");

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

  for(auto it_cfg = cfg.Begin(); it_cfg != cfg.End(); it_cfg++)
  {
    ROS_WARN("%s", (*it_cfg).first.c_str());
  }

  ROS_WARN("====================================== TEST ======================================");
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

std::vector<int> RobotParser::findPartialChain(const int endpoint_link)
{
    assert(endpoint_link < num_links && "endpoint_link is out of bounds");
    assert(endpoint_link >= 0 && "endpoint_link must be non-negative");

    int j_idx = link_parent_joint_idx[endpoint_link];
    std::vector<int> inv_chain;
    std::vector<bool> check;
    for (int k=0; k<num_joints; k++)
    {
        inv_chain.push_back(-1);
        check.push_back(false);
    }
    int p_idx=0;
    while (j_idx > 0)
    {
        assert(check[j_idx] == false && "detected loop in the kinematic tree");
        check[j_idx] = true;
        inv_chain[p_idx] = j_idx;
        j_idx = link_parent_joint_idx[ joint_parent_link_idx[j_idx] ]; // find the parent joint
        p_idx++;
    }
    int size = p_idx+1;
    assert(size <= num_joints && "partial chain can't be larger than the full chain");
    std::vector<int> partial_chain;
    partial_chain.resize(size);
    for (int i=0; i<size; i++)
    {
        partial_chain[i] = inv_chain[size-i-1];
    }
    partial_chain[0] = 0; // quick fix
    return partial_chain;
}


} // namespace salih_marangoz_thesis