#include <salih_marangoz_thesis/ceres_ik.h>
#include <salih_marangoz_thesis/utils/moveit_utils.h>

namespace salih_marangoz_thesis
{

CeresIK::CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh), rand_gen(rand_dev())
{
  // init planning_scene_monitor
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->setStateUpdateFrequency(100);
  //planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  // init visual_tools
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(planning_scene_monitor->getRobotModel()->getModelFrame(), "/ceres_ik_visual_markers"));

  // TODO
  endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1, &CeresIK::subscriberCallback, this);
  vis_pub = priv_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  marker_array_pub = priv_nh.advertise<visualization_msgs::MarkerArray>("visualization_array", 0);

  loop(); // TODO
}

void CeresIK::subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg)
{
  auto given_endpoint = msg->pose;
  ROS_WARN_THROTTLE(1.0, "Endpoint received. x: %lf, y: %lf, z: %lf", given_endpoint.position.x, given_endpoint.position.y, given_endpoint.position.z);
  endpoint.x() =  given_endpoint.position.x;
  endpoint.y() =  given_endpoint.position.y;
  endpoint.z() =  given_endpoint.position.z;
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
    //robot_state.setToRandomPositions();
    //robot_state = getCurrentRobotState();

    if (!update(robot_state))
    {
      ROS_ERROR("Can't find a solution!");
      r.sleep();
      continue;
    }

    visual_tools->publishRobotState(robot_state);
    r.sleep();
  }
}

bool CeresIK::update(moveit::core::RobotState &current_state)
{
  ROS_INFO_ONCE("Number of variables in the robot state: %d", (int)(current_state.getVariableCount()) );
  if (robot::num_variables != current_state.getVariableCount())
  {
    ROS_FATAL_ONCE("Number of variables does not match!");
    exit(-1);
  }

  double* variable_positions = current_state.getVariablePositions();

  // Generate target_positions (this is the init state and this will be optimized)
  double target_positions[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    const int variable_i = robot::joint_idx_to_variable_idx[joint_i];
    target_positions[i] = variable_positions[variable_i];

    // Add noise to the init state to avoid gimball lock, etc.
    double noise = 1.0; // TODO: 0.1
    if (noise>0)
    {
      double sampling_min = target_positions[i]-noise;
      if (sampling_min<robot::joint_min_position[joint_i]) sampling_min=robot::joint_min_position[joint_i];
      double sampling_max = target_positions[i]+noise;
      if (sampling_max>robot::joint_max_position[joint_i]) sampling_max=robot::joint_max_position[joint_i];
      std::uniform_real_distribution<> dis(sampling_min, sampling_max);
      target_positions[i] = dis(rand_gen);
    }
  }

  // Generate const_target_positions (this a noise-free copy of target_positions and should not be modified!)
  double const_target_positions[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    const int variable_i = robot::joint_idx_to_variable_idx[joint_i];
    const_target_positions[i] = variable_positions[i];
  }

  // Generate target centers. Assuming that center=(max+min)/2
  double target_centers[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    target_centers[i] = 0.5 * (robot::joint_max_position[joint_i] + robot::joint_min_position[joint_i]);
  }

  // Generate joint_idx_to_target_idx
  int joint_idx_to_target_idx[robot::num_joints];
  for (int i=0; i<robot::num_joints; i++) joint_idx_to_target_idx[i] = -1; // set to -1 by default
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    joint_idx_to_target_idx[joint_i] = i;
  }

  // TODO: Find the partial kinematic tree for the endpoint goal
  // int current_joint_idx = robot::endpoint_joint_idx;
  // .....


  ceres::Problem problem;

  //Eigen::Vector3d endpoint;
  //endpoint << 0.3 , 0.2 , 0.2;
  ceres::CostFunction* endpoint_goal = EndpointGoal::Create(endpoint, joint_idx_to_target_idx, variable_positions);
  problem.AddResidualBlock(endpoint_goal, nullptr /* squared loss */, target_positions);

  ceres::CostFunction* center_joints_goal = CenterJointsGoal::Create(target_centers);
  ceres::CauchyLoss *center_joints_loss = new ceres::CauchyLoss(0.1); // goal weight
  problem.AddResidualBlock(center_joints_goal, center_joints_loss, target_positions);

  ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(const_target_positions);
  ceres::CauchyLoss *minimal_joint_displacement_loss = new ceres::CauchyLoss(0.5); // goal weight
  problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, target_positions);

  // Target min/max constraints
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    if (!robot::joint_is_position_bounded[joint_i])
    {
      ROS_WARN("Target is not bounded. This can cause bad solutions!");
      continue;
    }
    problem.SetParameterLowerBound(target_positions, i, robot::joint_min_position[joint_i]); 
    problem.SetParameterUpperBound(target_positions, i, robot::joint_max_position[joint_i]); 
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_type = ceres::TRUST_REGION; // LINE_SEARCH methods don't support bounds
  options.minimizer_progress_to_stdout = false;
  // experimental
  //options.preconditioner_type = ceres::SUBSET;
  options.jacobi_scaling = true; // TODO: this was used in bio_ik, I think
  options.use_nonmonotonic_steps = true;
  options.use_approximate_eigenvalue_bfgs_scaling = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  // Update robot state
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[i];
    const int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];
    variable_positions[variable_idx] = target_positions[i];
  }
  current_state.update(true); // TODO: can be faster with: updateLinkTransforms()

  // collision debug
  auto marker_array = utils::visualizeCollisions<double>(current_state, joint_idx_to_target_idx, target_positions, variable_positions);
  marker_array_pub.publish(marker_array);

  return true;
}


} // namespace salih_marangoz_thesis