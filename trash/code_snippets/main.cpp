
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <bio_ik/bio_ik.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include <chrono>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    exit(-1);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      true /* skip octomap monitor */); // TODO: didn't work
  //planning_scene_monitor->setStateUpdateFrequency(100); // TODO
  planning_scene_monitor->startStateMonitor();

  collision_detection::AllowedCollisionMatrix acm;
  bool is_acm_init = false;

  auto begin = std::chrono::high_resolution_clock::now();
  int count=0;
  ros::Rate r(100.0);
  while (ros::ok())
  {
    //planning_scene_monitor->waitForCurrentRobotState(ros::Time::now()); // <---------------------- BLOCKING

    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
    //robot_model::RobotModelConstPtr model = ls->getRobotModel();

    auto current_state_ = planning_scene_monitor->getStateMonitor()->getCurrentState();
    current_state_->updateCollisionBodyTransforms();
    
    if (!is_acm_init)
    {
      acm = ls->getAllowedCollisionMatrix();

      std::vector<std::string> names;
      acm.getAllEntryNames(names);

      ROS_WARN("Before modifications:");
      acm.print(std::cout);

      for (int i=0; i<names.size(); i++)
      {
        for (int j=0; j<names.size(); j++)
        {
          if (names[i].find("arm") != std::string::npos && names[j].find("arm") != std::string::npos) // found
          {
            acm.setEntry(names[i], names[j], true);
          }
          else if (names[i].find("head") != std::string::npos && names[j].find("head") != std::string::npos) // found
          {
            acm.setEntry(names[i], names[j], true);
          }
          /*
          else if (names[i].find("head") != std::string::npos && names[j].find("arm") != std::string::npos) // found
          {
            acm.setEntry(names[i], names[j], false);
          }
          else if (names[i].find("arm") != std::string::npos && names[j].find("head") != std::string::npos) // found
          {
            acm.setEntry(names[i], names[j], false);
          }
          */
        }
      }

      ROS_WARN("After modifications:");
      acm.print(std::cout);

      is_acm_init = true;
    }

    double dist = ls->getCollisionEnv()->distanceSelf(*current_state_, acm);
    ROS_WARN("Dist-self: %f", dist);

    dist = ls->getCollisionEnv()->distanceRobot(*current_state_, acm);
    ROS_WARN("Dist-robo: %f", dist);

    count++;
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - begin);
    if (elapsed.count() > 1e9) // 1e9
    {
      begin = std::chrono::high_resolution_clock::now();
      ROS_WARN("Hz: %d", count);
      //ROS_WARN("Dist: %f", dist);
      count = 0;
    }
    //r.sleep(); // <---------------------- BLOCKING
  }

  return 0;
}

/*
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                  robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, 
                  const double *ik_solution)
{
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  ROS_WARN("%s\n", group->getName().c_str());
  return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName()));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state. One way to do this is to start an AsyncSpinner beforehand.
  spinner.start();

  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  static const std::string PLANNING_GROUP = "head";
  static const std::string ROBOT_DESCRIPTION = "robot_description";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  auto robot_model = robot_model_loader.getModel();
  auto joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);
  auto tip_names = joint_model_group->getSolverInstance()->getTipFrames();
  robot_state::RobotState robot_state_ik(robot_model);
  ROS_INFO("Model frame: %s", robot_model_loader.getModel()->getModelFrame().c_str());

  planning_scene::PlanningScene scene(robot_model); // USE MONITOR INSTEAD. THIS IS NOT RECOMMENDED!

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_loader.getModel()->getModelFrame(),"/moveit_visual_markers"));

  robot_state::GroupStateValidityCallbackFn constraint_fn;
  constraint_fn = boost::bind(&isStateValid, &scene, _1, _2, _3);

  //---------------------------------------------------------------------------------------------

  std::random_device rd;  // Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-0.5, 0.5);

  while(ros::ok())
  {
  tf2::Vector3 target(dis(gen)+0.5,dis(gen), dis(gen)+1);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = target.getX();
  marker.pose.position.y = target.getY();
  marker.pose.position.z = target.getZ();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );

  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.replace = true;
  ik_options.return_approximate_solution = false;

  auto* lookat_goal = new bio_ik::LookAtGoal();
  lookat_goal->setLinkName("head_link6");
  lookat_goal->setTarget(target);
  lookat_goal->setAxis(tf2::Vector3(0.0, 0.0, 1.0));
  ik_options.goals.emplace_back(lookat_goal);

  auto* avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
  ik_options.goals.emplace_back(avoid_joint_limits_goal);

  auto* minimal_displacement_goal = new bio_ik::MinimalDisplacementGoal();
  minimal_displacement_goal->setWeight(1.0);
  ik_options.goals.emplace_back(minimal_displacement_goal);

  auto* pos_goal = new bio_ik::PositionGoal();
  pos_goal->setLinkName("head_link6");
  pos_goal->setWeight(1);
  //pos_goal->setPosition(tf2::Vector3(1.0, 0.0, 1.0));
  //ik_options.goals.emplace_back(pos_goal);

  //geometry_msgs::Pose pos;
  //geometry_msgs::Vector3 size;
  //pos.position.x = 1;
  //pos.position.y = 0;
  //pos.position.z = 1;
  //size.x = 0.1; size.y = 0.1; size.z = 0.1; 
  //visual_tools_->publishCuboid(pos, size);

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  robot_state_ik.copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  bool found_ik = robot_state_ik.setFromIK(
                joint_model_group,           // active joints
                EigenSTL::vector_Isometry3d(), // no explicit poses here
                std::vector<std::string>(),  // no end effector links here
                10000, 10.0,                      // take values from YAML file
                constraint_fn, // moveit::core::GroupStateValidityCallbackFn(),
                ik_options       // goals
              );

  if (found_ik)
  {
    visual_tools_->publishRobotState(robot_state_ik);
    ros::spinOnce();

    robot_state_ik.copyJointGroupPositions(joint_model_group, joint_values);

    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // execute ----------------------
    
    std::map<std::string, double> goal_joint_values;
    std::vector<double> ik_joint_values;
    //robot_state_ik.copyJointGroupPositions(joint_model_group, ik_joint_values);
    //const std::vector<std::string> &joint_names_2 = joint_model_group->getJointModelNames();
    for(std::size_t i = 0; i < joint_names.size(); ++i) { // TODO cleanly remove the fixed joints (first and last) from the list
      goal_joint_values[joint_names[i]] = joint_values[i];
    }
    

    //move_group_interface.setGoalJointTolerance(0.5);

    move_group_interface.setStartStateToCurrentState(); //robot_state::RobotState start_state(*move_group_interface.getCurrentState()); move_group_interface.setStartState(start_state);
    move_group_interface.setJointValueTarget(goal_joint_values);
    //move_group_interface.setGoalOrientationTolerance(0.01);
    //move_group_interface.setGoalPositionTolerance(0.01);

    //move_group_interface.setPlanningTime(2.0);
    //move_group_interface.setNumPlanningAttempts(100);

    // Plan and execute
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //auto plan_result = move_group_interface.plan(my_plan);
    //move_group_interface.execute(my_plan);
    move_group_interface.move();

  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  ros::Duration d(1.0);
  d.sleep();
  }

  ros::shutdown();
  return 0;
}
*/