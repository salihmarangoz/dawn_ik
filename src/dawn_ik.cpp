#include <dawn_ik/dawn_ik.h>

namespace dawn_ik
{

DawnIK::DawnIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh), rand_gen(rand_dev())
{
  ik_goal_msg = boost::make_shared<dawn_ik::IKGoal>();
  ik_goal_msg->mode = IKGoal::MODE_0; // idle

  robot_monitor = std::make_shared<RobotMonitor>(nh, priv_nh);

  joint_controller = std::make_shared<JointTrajectoryControlInterface>(nh);

  solver_summary_pub = priv_nh.advertise<dawn_ik::SolverSummary>("solver_summary", 2);

  loop_thread = new boost::thread(boost::bind(&DawnIK::loopThread, this)); // consumer
  ik_goal_sub = priv_nh.subscribe("ik_goal", 1, &DawnIK::goalCallback, this); // producer

  // TODO
  endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1, &DawnIK::subscriberCallback, this);
}

DawnIK::~DawnIK()
{
  loop_thread->join();
  delete loop_thread;
}

void DawnIK::goalCallback(const dawn_ik::IKGoalPtr &msg)
{
  std::scoped_lock(ik_goal_mutex);
  ik_goal_msg = msg;
  ROS_INFO_ONCE("First IK Goal Message Received!");
}

// TODO: remove this function
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
  ros::Rate r(20); // TODO: param
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
  JointLinkCollisionStateConstPtr state = robot_monitor->getState();
  const std::vector<CollisionObject*> int_objects = robot_monitor->getInternalObjects();

   // TODO: DONT FORGET TO FIX THIS MESS. ORDER IS NOT GUARANTEED !!!!!!
  const double* variable_positions = state->joint_state.position.data();
  const double* variable_velocities = state->joint_state.velocity.data();

  // Generate target_positions (this is the init state and this will be optimized)
  double target_positions[robot::num_targets];
  double target_velocities[robot::num_targets];
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    const int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];
    
    target_positions[target_idx] = variable_positions[variable_idx];
    target_velocities[target_idx] = variable_velocities[variable_idx];

    // Add noise to the init state to avoid gimball lock, etc.
    double noise = 0.0; // TODO: 0.1
    if (noise>0)
    {
      double sampling_min = target_positions[target_idx]-noise;
      if (sampling_min<robot::joint_min_position[joint_idx]) sampling_min=robot::joint_min_position[joint_idx];
      double sampling_max = target_positions[target_idx]+noise;
      if (sampling_max>robot::joint_max_position[joint_idx]) sampling_max=robot::joint_max_position[joint_idx];
      std::uniform_real_distribution<> dis(sampling_min, sampling_max);
      target_positions[target_idx] = dis(rand_gen);
    }
  }

  // Generate const_target_positions (this a noise-free copy of target_positions and should not be modified!)
  double const_target_positions[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    const int variable_i = robot::joint_idx_to_variable_idx[joint_i];
    const_target_positions[i] = variable_positions[variable_i];
  }

  // TODO: Find the partial kinematic tree for the endpoint goal. Move this to the robot parser!!!!!!!!!!!!!!!!!

  ceres::Problem problem;

  // ========== Preferred Joint Position Goal ==========
  ceres::CostFunction* preferred_joint_position_goal = PreferredJointPositionGoal::Create();
  ceres::TukeyLoss *preferred_joint_position_loss = new ceres::TukeyLoss(0.1); // goal weight
  problem.AddResidualBlock(preferred_joint_position_goal, preferred_joint_position_loss, target_positions);
  //problem.AddResidualBlock(preferred_joint_position_goal, robot::createPreferredJointPositionLoss(), target_positions); // TODO

  // ========== Minimal Joint Displacement Goal ==========
  //ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(const_target_positions);
  //ceres::CauchyLoss *minimal_joint_displacement_loss = new ceres::CauchyLoss(0.1); // goal weight
  //problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, target_positions);

  // ================== Endpoint Goal ==================
  ceres::CostFunction* endpoint_goal = EndpointGoal::Create(endpoint, direction, variable_positions);
  //ceres::HuberLoss *endpoint_loss = new ceres::HuberLoss(1.0); // goal weight
  ceres::RelaxedIKLoss *endpoint_loss = new ceres::RelaxedIKLoss(0.5); // goal weight
  problem.AddResidualBlock(endpoint_goal, endpoint_loss, target_positions);
  //problem.AddResidualBlock(endpoint_goal, robot::createEndpointLoss(), target_positions);

  // ============= Collision Avoidance Goal ============
  ceres::CostFunction* collision_avoidance_goal = CollisionAvoidanceGoal::Create(variable_positions,
                                                                                 state->collision_state.int_pair_a.data(),
                                                                                 state->collision_state.int_pair_b.data(),
                                                                                 state->collision_state.int_pair_a.size(),
                                                                                 int_objects);
  //ceres::HuberLoss *collision_avoidance_loss = new ceres::HuberLoss(1.0); // goal weight
  problem.AddResidualBlock(collision_avoidance_goal, nullptr, target_positions);
  //problem.AddResidualBlock(collision_avoidance_goal, robot::createCollisionAvoidanceLoss(), target_positions);



  // Target min/max constraints
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    if (!robot::joint_is_position_bounded[joint_i])
    {
      ROS_WARN("Target is not bounded. This can cause bad solutions!");
      continue;
    }

    // TODO: ALTERNATIVE WAY? IK GOAL JOINT SPEED MULTIPLIER? ZERO TO FIX THE JOINT?
    double min_val = const_target_positions[i] - 0.1;
    double max_val = const_target_positions[i] + 0.1;
    if (robot::joint_min_position[joint_i] > min_val) min_val = robot::joint_min_position[joint_i];
    if (robot::joint_max_position[joint_i] < max_val) max_val = robot::joint_max_position[joint_i];
    problem.SetParameterLowerBound(target_positions, i, min_val); 
    problem.SetParameterUpperBound(target_positions, i, max_val); 

    // STANDARD WAY OF SETTING JOINT LIMITS
    //problem.SetParameterLowerBound(target_positions, i, robot::joint_min_position[joint_i]); 
    //problem.SetParameterUpperBound(target_positions, i, robot::joint_max_position[joint_i]); 
  }

  ceres::Solver::Options options;
  robot::setSolverOptions(options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (solver_summary_pub.getNumSubscribers() > 0)
  {
    dawn_ik::SolverSummary summary_msg;
    summary_msg.is_solution_usable = summary.IsSolutionUsable();
    summary_msg.total_time_in_seconds = summary.total_time_in_seconds;
    summary_msg.brief_report = summary.BriefReport();
    summary_msg.full_report = summary.FullReport();
    summary_msg.header = state->header;
    solver_summary_pub.publish(summary_msg);
  }

  if (!summary.IsSolutionUsable()) return false;

  // Publish output
  std::vector<std::string> target_names;
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    target_names.push_back(robot::joint_names[joint_idx]);
  }
  joint_controller->setJointPositions(target_names, target_positions);

  return true;
}


} // namespace dawn_ik