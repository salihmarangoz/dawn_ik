#include <dawn_ik/dawn_ik.h>

// TODO LIST
// - remove interactive marker feedback stuff
// - goals should be able to handle non-bounded positions
// - get loss functions from the parser. Ex: problem.AddResidualBlock(preferred_joint_position_goal, robot::createPreferredJointPositionLoss(), target_positions);
// - discarded unusable solution case. What to do in that case?
//     if (!summary.IsSolutionUsable()) return false;

namespace dawn_ik
{

DawnIK::DawnIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh), rand_gen(rand_dev())
{
  readParameters();

  for (int i=0; i<robot::num_joints; i++) joint_name_to_joint_idx[robot::joint_names[i]] = i;

  ik_goal_msg = boost::make_shared<dawn_ik::IKGoal>();
  ik_goal_msg->mode = IKGoal::MODE_0; // idle

  robot_monitor = std::make_shared<RobotMonitor>(nh, priv_nh);

  joint_controller = std::make_shared<JointTrajectoryControlInterface>(nh);

  solver_summary_pub = priv_nh.advertise<dawn_ik::SolverSummary>("solver_summary", 2);

  loop_thread = new boost::thread(boost::bind(&DawnIK::loopThread, this)); // consumer
  ik_goal_sub = priv_nh.subscribe("ik_goal", 1, &DawnIK::goalCallback, this); // producer

  //endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1, &DawnIK::subscriberCallback, this);
}

DawnIK::~DawnIK()
{
  loop_thread->join();
  delete loop_thread;
}

void
DawnIK::readParameters()
{
  priv_nh.param("update_rate", p_update_rate, 100.0);
  if (p_update_rate <= 0.0){ROS_ERROR("Invalid update_rate!"); p_update_rate = 100.0;}

  priv_nh.param("init_noise", p_init_noise, 0.01);
  if (p_init_noise < 0.0){ROS_ERROR("Invalid init_noise!"); p_init_noise = 0.0;} // disable on error

  priv_nh.param("max_step_size", p_max_step_size, 0.2);
  if (p_max_step_size < 0.0){ROS_ERROR("Invalid max_step_size!"); p_max_step_size = 0.0;} // disable on error
}

void DawnIK::goalCallback(const dawn_ik::IKGoalPtr &msg)
{
  std::scoped_lock(ik_goal_mutex);
  ik_goal_msg = msg;
  ROS_INFO_ONCE("First IK Goal Message Received!");
}

void DawnIK::subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg)
{
  auto given_endpoint = msg->pose;
  ROS_WARN_THROTTLE(1.0, "Endpoint received. x: %lf, y: %lf, z: %lf", given_endpoint.position.x, given_endpoint.position.y, given_endpoint.position.z);
  endpoint.x() =  given_endpoint.position.x;
  endpoint.y() =  given_endpoint.position.y;
  endpoint.z() =  given_endpoint.position.z;
  direction = Eigen::Quaterniond(given_endpoint.orientation.w, given_endpoint.orientation.x, given_endpoint.orientation.y, given_endpoint.orientation.z);
  ik_goal_msg->mode = IKGoal::MODE_1;
}

void DawnIK::loopThread()
{
  ros::Rate r(p_update_rate);
  while (ros::ok())
  {
    ik_goal_mutex.lock();
    dawn_ik::IKGoalPtr ik_goal_msg_copy = ik_goal_msg;
    ik_goal_mutex.unlock();

    if (ik_goal_msg_copy->mode != IKGoal::MODE_0)
    {
      update(ik_goal_msg_copy);
    }

    r.sleep();
  }
}

bool DawnIK::update(const dawn_ik::IKGoalPtr &ik_goal)
{
  //=================================================================================================
  // Get current state
  //=================================================================================================
  JointLinkCollisionStateConstPtr monitor_state = robot_monitor->getState();
  const std::vector<CollisionObject*> int_objects = robot_monitor->getInternalObjects();

  //endpoint.x() =  ik_goal->m1_x.value;
  //endpoint.y() =  ik_goal->m1_y.value;
  //endpoint.z() =  ik_goal->m1_z.value;
  //direction = Eigen::Quaterniond(given_endpoint.orientation.w, given_endpoint.orientation.x, given_endpoint.orientation.y, given_endpoint.orientation.z);

  std::vector<double> variable_positions;
  variable_positions.resize(robot::num_variables);
  std::vector<double> variable_velocities;
  variable_velocities.resize(robot::num_variables);
  for (int i=0; i< monitor_state->joint_state.name.size(); i++)
  {
    if(joint_name_to_joint_idx.find(monitor_state->joint_state.name[i]) == joint_name_to_joint_idx.end()) continue;
    int joint_idx = joint_name_to_joint_idx[monitor_state->joint_state.name[i]];
    int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];

    variable_positions[variable_idx] = monitor_state->joint_state.position[i];
    variable_velocities[variable_idx] = monitor_state->joint_state.velocity[i];
  }

  //=================================================================================================
  // Set up the problem inputs
  //=================================================================================================
  double optm_target_positions[robot::num_targets];  // -----> this will be the output. random noise can be added before the optimization.
  double curr_target_positions[robot::num_targets];  // -----> this is the original input. do not modify.
  double curr_target_velocities[robot::num_targets]; // -----> this is the original input. do not modify.
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    const int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];

    optm_target_positions[target_idx]  = variable_positions[variable_idx];
    curr_target_positions[target_idx]  = variable_positions[variable_idx];
    curr_target_velocities[target_idx] = variable_velocities[variable_idx];

    // Add noise to the init state to quickly escape form gimball locks, etc.
    if (p_init_noise > 0)
    {
      double sampling_min = curr_target_positions[target_idx]-p_init_noise;
      if (sampling_min<robot::joint_min_position[joint_idx]) sampling_min=robot::joint_min_position[joint_idx];
      double sampling_max = curr_target_positions[target_idx]+p_init_noise;
      if (sampling_max>robot::joint_max_position[joint_idx]) sampling_max=robot::joint_max_position[joint_idx];
      std::uniform_real_distribution<> dis(sampling_min, sampling_max);
      optm_target_positions[target_idx] = dis(rand_gen);
    }
  }

  //=================================================================================================
  // Create the shared block (for read-only information sharing to the objectives)
  //=================================================================================================
  dawn_ik::SharedBlock shared_block(ik_goal,
                                    solver_history,
                                    joint_name_to_joint_idx,
                                    variable_positions,
                                    variable_velocities,
                                    curr_target_positions,
                                    curr_target_velocities,
                                    monitor_state,
                                    int_objects);

  //=================================================================================================
  // Set up the optimization problem
  //=================================================================================================
  ceres::Problem problem;
  ceres::Solver::Options options;
  robot::setSolverOptions(options);
  ceres::Solver::Summary summary;

  // ========== Preferred Joint Position Goal ==========
  ceres::CostFunction* preferred_joint_position_goal = PreferredJointPositionGoal::Create(shared_block);
  //ceres::TukeyLoss *preferred_joint_position_loss = new ceres::TukeyLoss(0.25); // goal weight
  //ceres::CauchyLoss *preferred_joint_position_loss = new ceres::CauchyLoss(0.05); // goal weight
  // TODO
  double dynamic_preferred_joint_position_weight = shared_block.dist_to_target;
  if (dynamic_preferred_joint_position_weight > shared_block.ik_goal->m1_limit_dist)
  {
    dynamic_preferred_joint_position_weight = shared_block.ik_goal->m1_limit_dist;
  }
  // if (dynamic_preferred_joint_position_weight < shared_block.dist_to_target/2)
  // {
  //   dynamic_preferred_joint_position_weight = shared_block.dist_to_target/2;
  // }
  ceres::CauchyLoss *preferred_joint_position_loss = new ceres::CauchyLoss(dynamic_preferred_joint_position_weight*2); // goal weight
  problem.AddResidualBlock(preferred_joint_position_goal, preferred_joint_position_loss, optm_target_positions);

  // ========== Avoid Joint Limits Goal ==========
  ceres::CostFunction* avoid_joint_limits_goal = AvoidJointLimitsGoal::Create(shared_block);
  //ceres::TukeyLoss *avoid_joint_limits_loss = new ceres::TukeyLoss(0.1); // goal weight
  problem.AddResidualBlock(avoid_joint_limits_goal, nullptr, optm_target_positions);

  // ================== Endpoint Goal ==================
  ceres::CostFunction* endpoint_goal = EndpointGoal::Create(shared_block);
  //ceres::HuberLoss *endpoint_loss = new ceres::HuberLoss(1.0); // goal weight
  //ceres::RelaxedIKLoss *endpoint_loss = new ceres::RelaxedIKLoss(0.5); // goal weight
  problem.AddResidualBlock(endpoint_goal, nullptr, optm_target_positions);

  // ============= Collision Avoidance Goal ============
  ceres::CostFunction* collision_avoidance_goal = CollisionAvoidanceGoal::Create(shared_block);
  problem.AddResidualBlock(collision_avoidance_goal, nullptr, optm_target_positions);


  //=================================================================================================
  // Set parameter constraints
  //=================================================================================================

  // TODO
  //options.trust_region_strategy_type = DOGLEG;
  //options.use_nonmonotonic_steps = true;
  //options.preconditioner_type = JACOBI;
  //options.minimizer_type = LINE_SEARCH;
  //options.line_search_direction_type = BFGS;
  options.jacobi_scaling = false;

  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    if (!robot::joint_is_position_bounded[joint_idx]) continue;

    // TODO: EXPERIMENTAL PROBLEM BOUNDARIES
    if (p_max_step_size > 0.0 && options.minimizer_type == TRUST_REGION)
    {
      double min_val = curr_target_positions[target_idx] - p_max_step_size;
      double max_val = curr_target_positions[target_idx] + p_max_step_size;
      if (robot::joint_min_position[joint_idx] > min_val) min_val = robot::joint_min_position[joint_idx];
      if (robot::joint_max_position[joint_idx] < max_val) max_val = robot::joint_max_position[joint_idx];
      problem.SetParameterLowerBound(optm_target_positions, target_idx, min_val); 
      problem.SetParameterUpperBound(optm_target_positions, target_idx, max_val); 
    }
    else
    {
      // ========== Minimal Joint Displacement Goal ==========
      ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(shared_block);
      //ceres::CauchyLoss *minimal_joint_displacement_loss = new ceres::CauchyLoss(0.005); // goal weight
      ceres::TukeyLoss *minimal_joint_displacement_loss = new ceres::TukeyLoss(0.05); // goal weight
      problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, optm_target_positions);
    }

    // STANDARD WAY OF SETTING JOINT LIMITS
    //problem.SetParameterLowerBound(target_positions, i, robot::joint_min_position[joint_i]); 
    //problem.SetParameterUpperBound(target_positions, i, robot::joint_max_position[joint_i]); 
  }

  //=================================================================================================
  // Solve!!!
  //=================================================================================================
  ceres::Solve(options, &problem, &summary);

  if (solver_summary_pub.getNumSubscribers() > 0)
  {
    dawn_ik::SolverSummary summary_msg;
    summary_msg.is_solution_usable = summary.IsSolutionUsable();
    summary_msg.total_time_in_seconds = summary.total_time_in_seconds;
    summary_msg.brief_report = summary.BriefReport();
    summary_msg.full_report = summary.FullReport();
    summary_msg.header = monitor_state->header;
    solver_summary_pub.publish(summary_msg);
  }

  //=================================================================================================
  // Process the output
  //=================================================================================================

  // Publish output
  std::vector<std::string> target_names;
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    target_names.push_back(robot::joint_names[joint_idx]);
  }
  joint_controller->setJointPositions(target_names, optm_target_positions);

  // Keep history
  std::vector<double> optm_target_positions_v;
  for (int i=0; i<robot::num_targets; i++) optm_target_positions_v.push_back(optm_target_positions[i]);
  //for (int i=0; i<robot::num_targets; i++) optm_target_positions_v.push_back((optm_target_positions[i] - curr_target_positions[i])*0.5 + curr_target_positions[i]);
  solver_history.push(optm_target_positions_v);
  if (solver_history.size() > 4) solver_history.pop();

  return true;
}


} // namespace dawn_ik