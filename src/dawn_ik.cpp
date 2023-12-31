#include <dawn_ik/dawn_ik.h>
#include <std_msgs/Float64MultiArray.h>

namespace dawn_ik
{

DawnIK::DawnIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh), rand_gen(rand_dev())
{
  readParameters();

  problem_options.context = ceres::Context::Create();

  for (int i=0; i<robot::num_joints; i++) joint_name_to_joint_idx[robot::joint_names[i]] = i;

  ik_goal_msg = boost::make_shared<dawn_ik::IKGoal>();
  ik_goal_msg->mode = IKGoal::MODE_0; // idle

  robot_monitor = std::make_shared<RobotMonitor>(nh, priv_nh);

  joint_controller = std::make_shared<JointTrajectoryControlInterface>(nh);

  solver_summary_pub = priv_nh.advertise<dawn_ik::SolverSummary>("solver_summary", 2);
  debug_pub = priv_nh.advertise<std_msgs::Float64MultiArray>("debug", 2);

  loop_thread = new boost::thread(boost::bind(&DawnIK::loopThread, this)); // consumer
  ik_goal_sub = priv_nh.subscribe("ik_goal", 1, &DawnIK::goalCallback, this); // producer

#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  km_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(robot_model_);
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(),"/moveit_visual_markers"));
#endif
}

DawnIK::~DawnIK()
{
  loop_thread->join();
  delete loop_thread;
}

void DawnIK::readParameters()
{
  priv_nh.param("update_rate", p_update_rate, 100.0);
  if (p_update_rate <= 0.0){ROS_ERROR("Invalid update_rate!"); p_update_rate = 100.0;}

  priv_nh.param("init_noise", p_init_noise, 0.01);
  if (p_init_noise < 0.0){ROS_ERROR("Invalid init_noise!"); p_init_noise = 0.0;} // disable on error

  priv_nh.param("max_step_size", p_max_step_size, 0.01);
  if (p_max_step_size < 0.0){ROS_ERROR("Invalid max_step_size!"); p_max_step_size = 0.0;} // disable on error
}

void DawnIK::goalCallback(const dawn_ik::IKGoalPtr &msg)
{
  priv_nh.param("acc_loss_weight", acc_loss_weight, 10.0);
  std::scoped_lock(ik_goal_mutex);
  ik_goal_msg = msg;
  ROS_INFO_ONCE("First IK Goal Message Received!");
}

void DawnIK::loopThread()
{
  std::vector<std::string> target_names;
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    target_names.push_back(robot::joint_names[joint_idx]);
  }

  ros::Rate r(p_update_rate);
  while (ros::ok())
  {
    ik_goal_mutex.lock();

    dawn_ik::IKGoalPtr ik_goal_msg_copy = ik_goal_msg;
    ik_goal_mutex.unlock();



    if (ik_goal_msg_copy->mode != IKGoal::MODE_0)
    {
      // Random initialization is important to escape from DOF-lowering singularities. But it is possible to get noisy results with a limited time budget.
      // If we initialize using the previous joint state with added random noise, we can prevent these singularities, but we may not find a smooth solution with a limited time budget.
      // If we initialize using the previous joint state without noise, we can find a smooth solution quickly, but we it may take some time for the robot to escape these singularities.
      // Solution would be solving with both methods simultaneously (multi-threaded) and selecting the best solution.
      // TODO: split to threads, otherwise this loop cycle may exceed the limit
      command_history.clear();
      command_history = robot_monitor->getCommandHistory();
      auto latest_command = command_history.front();

      IKSolution ik_solution = update(ik_goal_msg_copy, true); // noisy init
      IKSolution ik_solution_without_noise = update(ik_goal_msg_copy, false); // clean init
      if (ik_solution.solver_summary.final_cost > ik_solution_without_noise.solver_summary.final_cost)
      {
        ik_solution = ik_solution_without_noise; // overwrite previous solution
      }

      joint_controller->setJointPositions(target_names, ik_solution.target_positions.data(), latest_command.position.data(), latest_command.velocity.data());
      

      // Keep history
      solver_history.push_front(ik_solution.target_positions);
      if (solver_history.size() >= 32) solver_history.pop_back();

      if (solver_history.size() >= 6) // todo
      {
        if (prev_pos.size() == 0)
        {
          prev_pos.resize(robot::num_targets, 0.0);
          prev_vel.resize(robot::num_targets, 0.0);
          prev_acc.resize(robot::num_targets, 0.0);
        }
        for (int i=0; i<robot::num_targets; i++)
        {
          // backward diff
          double sig_vel0 = solver_history[0][i] - solver_history[1][i];
          double sig_vel1 = solver_history[1][i] - solver_history[2][i];
          double sig_acc0 = sig_vel0 - sig_vel1;

          // exp weighted avg
          double alpha = 0.8;
          double new_pos = alpha*prev_pos[i] + (1-alpha)*solver_history[0][i];
          double new_vel = alpha*prev_vel[i] + (1-alpha)*(sig_vel0);
          double new_acc = alpha*prev_acc[i] + (1-alpha)*(sig_acc0);


          prev_pos[i] = new_pos;
          prev_vel[i] = new_vel;
          prev_acc[i] = new_acc;
        }
      }

    }
    ROS_INFO_THROTTLE(0.25, "cycle time: %f", r.cycleTime().toSec());
    r.sleep();
  }
}

IKSolution DawnIK::update(const dawn_ik::IKGoalPtr &ik_goal, bool noisy_initialization)
{
  //=================================================================================================
  // Get current state
  //=================================================================================================
  JointLinkCollisionStateConstPtr monitor_state = robot_monitor->getState();
  const std::vector<CollisionObject*> int_objects = robot_monitor->getInternalObjects();

  //auto command_history = robot_monitor->getCommandHistory();
  auto latest_command = command_history.front();
  std::vector<double> variable_positions(robot::num_variables, 0.0);
  std::vector<double> variable_velocities(robot::num_variables, 0.0);
  for (int i=0; i< monitor_state->joint_state.name.size(); i++)
  {
    if(joint_name_to_joint_idx.find(monitor_state->joint_state.name[i]) == joint_name_to_joint_idx.end()) continue;
    int joint_idx = joint_name_to_joint_idx[monitor_state->joint_state.name[i]];
    int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];

    variable_positions[variable_idx] = monitor_state->joint_state.position[i];
    
    // if (monitor_state->joint_state.velocity.size() > i) TODO: PRINT WARNING!
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

    if(solver_history.size() > 0)
    {
      optm_target_positions[target_idx]    = solver_history[0].at(target_idx);
      curr_target_positions[target_idx]    = solver_history[0].at(target_idx);
    }
    else
    {
      optm_target_positions[target_idx]    = latest_command.position[target_idx];
      curr_target_positions[target_idx]    = latest_command.position[target_idx];
    }
    curr_target_velocities[target_idx]   = latest_command.velocity[target_idx];
    // Add noise to the init state to quickly escape form gimball locks, etc.
    if (p_init_noise > 0 && noisy_initialization)
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
                                    int_objects,
                                    command_history,
                                    prev_pos);
#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
  shared_block.robot_model_ = robot_model_;
  shared_block.km_ = km_;
  shared_block.visual_tools_ = visual_tools_;
#endif
  //=================================================================================================
  // Set up the optimization problem
  //=================================================================================================
  ceres::Problem problem(problem_options);
  ceres::Solver::Options options;
  robot::setSolverOptions(options);

  // ========== Preferred Joint Position Goal ==========
  ceres::CostFunction* preferred_joint_position_goal = PreferredJointPositionGoal::Create(shared_block);

  double dynamic_preferred_joint_position_weight = shared_block.dist_to_target;
  if (dynamic_preferred_joint_position_weight > shared_block.ik_goal->m1_limit_dist)
  {
    dynamic_preferred_joint_position_weight = shared_block.ik_goal->m1_limit_dist;
  }
  ceres::CauchyLoss *preferred_joint_position_loss = new ceres::CauchyLoss(0.1); // goal weight
  problem.AddResidualBlock(preferred_joint_position_goal, preferred_joint_position_loss, optm_target_positions);

  // ========== Avoid Joint Limits Goal ==========
  ceres::CostFunction* avoid_joint_limits_goal = AvoidJointLimitsGoal::Create(shared_block);
  problem.AddResidualBlock(avoid_joint_limits_goal, nullptr, optm_target_positions);

  // ================== Endpoint Goal ==================
  if (ik_goal->m1_weight > 0.0 || ik_goal->m2_weight > 0.0)
  {
    ceres::CostFunction* endpoint_goal = EndpointGoal::Create(shared_block);
    problem.AddResidualBlock(endpoint_goal, nullptr, optm_target_positions);
  }

  // ================== Look at Goal ==================
  if (ik_goal->m3_weight > 0.0 )
  {
    ceres::CostFunction* look_at_goal = LookAtGoal::Create(shared_block);
    problem.AddResidualBlock(look_at_goal, nullptr, optm_target_positions);
  }

  // ============= Collision Avoidance Goal ============
  if (monitor_state->collision_state.int_pair_a.size() > 0) // skip this objective if proximity is the case
  {
    ceres::CostFunction* collision_avoidance_goal = CollisionAvoidanceGoal::Create(shared_block);
    problem.AddResidualBlock(collision_avoidance_goal, nullptr, optm_target_positions);
  }

  // ========== Minimal Joint Displacement Goal ==========
  ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(shared_block);
  ceres::LossFunction *minimal_joint_displacement_loss = new ceres::ScaledLoss(nullptr, 1.75, ceres::TAKE_OWNERSHIP); // goal weight
  problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, optm_target_positions);

#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
  ceres::CostFunction* manipulability_goal = ManipulabilityGoal::Create(shared_block);
  ceres::HuberLoss *manipulability_loss = new ceres::HuberLoss(2.0); // goal weight (0.25)
  //ceres::LossFunction *manipulability_scaled_loss = new ceres::ScaledLoss(manipulability_loss, 0.1, ceres::TAKE_OWNERSHIP); // goal weight
  problem.AddResidualBlock(manipulability_goal, manipulability_loss, optm_target_positions);
#endif

  //=================================================================================================
  // Set parameter constraints
  //=================================================================================================
  for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[target_idx];
    if (!robot::joint_is_position_bounded[joint_idx]) continue;

    double qddot_max =  utils::getBoundedValue((shared_block.command_history[0].acceleration[target_idx]) + (10*0.1), M_PI);
    double qddot_min =  utils::getBoundedValue((shared_block.command_history[0].acceleration[target_idx]) - (10*0.1), M_PI);
    double qdot_max = utils::getBoundedValue((shared_block.command_history[0].velocity[target_idx]) + qddot_max*(0.1), M_PI);
    double qdot_min = utils::getBoundedValue((shared_block.command_history[0].velocity[target_idx]) + qddot_min*(0.1), M_PI);

    if (p_max_step_size > 0.0 && options.minimizer_type == TRUST_REGION)
    {
      ///////////////////////////////////////////////////////////////////
      double min_step = -p_max_step_size;
      double max_step = p_max_step_size;

      if (solver_history.size() > 30)
      {
        double jerk_limit = 1.0* 500.0*0.01*0.01*0.01; // 500rad/s^3
        double acc_limit = 1.0 * 20.0*0.01*0.01; // 20rad/s^2
        double vel_limit = 0.6* 1.0*0.01; // 1rad/s
        double limited_vel = utils::getBoundedValue(prev_vel[target_idx], vel_limit);
        double limited_acc = utils::getBoundedValue(prev_acc[target_idx], acc_limit);

        min_step = limited_vel + limited_acc - jerk_limit;
        max_step = limited_vel + limited_acc + jerk_limit;
        ROS_WARN_THROTTLE(0.1, "%f", prev_acc[target_idx]);
      }
      ///////////////////////////////////////////////////////////////////

      double min_val = curr_target_positions[target_idx] + min_step; // -3.14*0.05; //p_max_step_size; //qdot_min*0.1; //p_max_step_size;
      double max_val = curr_target_positions[target_idx] + max_step; // +3.14*0.05; //qdot_max*0.1; //p_max_step_size;
      //double min_val = curr_target_positions[target_idx] - 0.1;
      //double max_val = curr_target_positions[target_idx] + 0.1;

      if (robot::joint_min_position[joint_idx] > min_val) min_val = robot::joint_min_position[joint_idx];
      if (robot::joint_max_position[joint_idx] < max_val) max_val = robot::joint_max_position[joint_idx];
      problem.SetParameterLowerBound(optm_target_positions, target_idx, min_val); 
      problem.SetParameterUpperBound(optm_target_positions, target_idx, max_val); 
    }
    else
    {
      // ========== Minimal Joint Displacement Goal ==========
      ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(shared_block);
      ceres::TukeyLoss *minimal_joint_displacement_loss = new ceres::TukeyLoss(0.05); // goal weight
      problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, optm_target_positions);
    }

    // STANDARD WAY OF SETTING JOINT LIMITS (suitable for globally solving)
    //problem.SetParameterLowerBound(target_positions, i, robot::joint_min_position[joint_i]); 
    //problem.SetParameterUpperBound(target_positions, i, robot::joint_max_position[joint_i]); 
  }

  //=================================================================================================
  // Solve!!!
  //=================================================================================================
  ceres::Solver::Summary summary;
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

  // DEBUG
  std_msgs::Float64MultiArray debug_msg; 
  if (shared_block.solver_history.size() > 2)
  {
    int idx = 1;
    double current_vel = (optm_target_positions[idx] - shared_block.solver_history[0].at(idx)) / 0.01;
    double last_vel = (shared_block.solver_history[0].at(idx) - shared_block.solver_history[1].at(idx)) / 0.01 ;
    double acc = (current_vel - last_vel) / 0.01;
    debug_msg.data.push_back(acc);
    debug_pub.publish(debug_msg);
  }


  //=================================================================================================
  // Return the solution
  //=================================================================================================
  IKSolution solution;
  solution.solver_summary = summary;
  for (int i=0; i<robot::num_targets; i++) solution.target_positions.push_back(optm_target_positions[i]);
  return solution;
}


} // namespace dawn_ik
