#include <salih_marangoz_thesis/robot_parser.h>

namespace salih_marangoz_thesis
{

RobotParser::RobotParser(ros::NodeHandle &nh, ros::NodeHandle &priv_nh) : nh(nh), priv_nh(priv_nh)
{

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

  /*
  // Load Moveit robot model
  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
  kinematic_model = robot_model_loader->getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Follow the kinematic tree
  const moveit::core::JointModel* current_joint = kinematic_model->getRootJoint();
  ROS_INFO("Joint: %s", current_joint->getName().c_str());

  while (true)
  {
    // One child link is possible
    const moveit::core::LinkModel* current_link = current_joint->getChildLinkModel();
    ROS_INFO("Link: %s", current_link->getName().c_str());

    // Get link transform
    const Eigen::Isometry3d current_link_transform = current_link->getJointOriginTransform();
    ROS_INFO_STREAM("translation: " << current_link_transform.translation() << " " << current_link_transform.translation().isZero());
    ROS_INFO_STREAM("rotation: " << current_link_transform.rotation() << " " << current_link_transform.rotation().isIdentity());

    // Multiple joints are possible. But here we will pick the first one.
    const std::vector<const moveit::core::JointModel*> next_joints = current_link->getChildJointModels();
    if (next_joints.size() <= 0) break; // end of the kinematic tree
    current_joint = next_joints[0];
    ROS_INFO("Joint: %s", current_joint->getName().c_str());
  }
  */

  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    exit(-1);
  }

  // Start the planning scene monitor
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("endpoint_pose", 1, true);


  ros::Rate r(1.0);
  while (ros::ok())
  {
    planning_scene_monitor->waitForCurrentRobotState(ros::Time::now());
    planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
    moveit::core::RobotState current_state = lps->getCurrentState(); // copy the current state

    geometry_msgs::PoseStamped endpoint = forwardKinematics(current_state);
    pub.publish(endpoint);
    r.sleep();
  }

}



} // namespace salih_marangoz_thesis