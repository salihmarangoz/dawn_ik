#include <salih_marangoz_thesis/ceres_ik.h>

namespace salih_marangoz_thesis
{

CeresIK::CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh)
{
  // init planning_scene_monitor
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->setStateUpdateFrequency(100);
  //planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  // init visual_tools
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(planning_scene_monitor->getRobotModel()->getModelFrame(), "/ceres_ik_visual_markers"));

  loop(); // TODO
}

moveit::core::RobotState CeresIK::getCurrentRobotState()
{
  // TODO: this method is slow... 
  planning_scene_monitor->waitForCurrentRobotState(ros::Time(0));
  planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
  return lps->getCurrentState(); // copy the current state
}

// TODO: this is just a dummy loop function. this will run on a separate thread!
void CeresIK::loop()
{
  
  moveit::core::RobotState robot_state = getCurrentRobotState();

  ros::Rate r(100);
  while (ros::ok())
  {
    if (!update(robot_state))
    {
      ROS_ERROR("Can't find a solution!");
      r.sleep();
      continue;
    }
    //robot_state.setToRandomPositions();
    robot_state = getCurrentRobotState();

    robot_state.update(); // TODO: can be faster with: updateLinkTransforms()
    visual_tools->publishRobotState(robot_state);
    r.sleep();
  }
}

bool CeresIK::update(moveit::core::RobotState &current_state)
{
  ROS_INFO_ONCE("Number of variables in the robot state: %d", current_state.getVariableCount());
  if (robot::num_variables != current_state.getVariableCount())
  {
    ROS_FATAL_ONCE("Number of variables does not match!");
    exit(-1);
  }


  double* variable_positions = current_state.getVariablePositions();
  




  return true;
}




} // namespace salih_marangoz_thesis