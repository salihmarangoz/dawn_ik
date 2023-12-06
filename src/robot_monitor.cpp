#include <dawn_ik/robot_monitor.h>

namespace dawn_ik
{

CollisionCallBackCollect::CollisionCallBackCollect(){}

bool CollisionCallBackCollect::collide(CollisionObject* o1, CollisionObject* o2) {
  collision_pairs.push_back(std::make_pair(o1, o2));
  return false;
}

size_t CollisionCallBackCollect::numCollisionPairs() const {
  return collision_pairs.size();
}

const std::vector<CollisionCallBackCollect::CollisionPair>&
CollisionCallBackCollect::getCollisionPairs() const {
  return collision_pairs;
}

void CollisionCallBackCollect::init() { collision_pairs.clear(); }

bool CollisionCallBackCollect::exist(const CollisionPair& pair) const {
  return std::find(collision_pairs.begin(), collision_pairs.end(), pair) != collision_pairs.end();
}

////////////////////////////////////////////////////////////////////


RobotMonitor::RobotMonitor(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh)
{
  visualization_pub = priv_nh.advertise<visualization_msgs::MarkerArray>( "robot_monitor_visualization", 0 );
  joint_state_sub = nh.subscribe("joint_states", 2, &RobotMonitor::jointStateCallback, this);
  link_state_thread = new boost::thread(boost::bind(&RobotMonitor::updateLinkThread, this));
  collision_state_thread = new boost::thread(boost::bind(&RobotMonitor::updateCollisionThread, this));
  visualization_thread = new boost::thread(boost::bind(&RobotMonitor::updateVisualizationThread, this));

  joint_trajctrl_state_sub = nh.subscribe("joint_trajectory_state", 2, &RobotMonitor::jointTrajectoryControllerStateCallback, this);

  acc_jerk_pub = priv_nh.advertise<sensor_msgs::JointState>("calc_acc_jerk_command", 0);

  auto command = Command(cycle_time);
  command.relative_time_stamp = 0.0;
  command.time_diff = cycle_time;
  command_history.push_front(command);
  command.relative_time_stamp = command.relative_time_stamp + cycle_time;
  command.time_diff = cycle_time;
  command_history.push_front(command);
  command.relative_time_stamp = command.relative_time_stamp + cycle_time;
  command_history.push_front(command);
}

RobotMonitor::~RobotMonitor()
{
  link_state_thread->join();
  delete link_state_thread;
  collision_state_thread->join();
  delete collision_state_thread;
}

const JointLinkCollisionStateConstPtr
RobotMonitor::getState()
{
  last_joint_link_collision_state_mtx.lock();
  JointLinkCollisionStateConstPtr tmp = last_joint_link_collision_state_msg;
  last_joint_link_collision_state_mtx.unlock();
  return tmp;
}

void RobotMonitor::jointTrajectoryControllerStateCallback(const control_msgs::JointTrajectoryControllerStatePtr & msg)
{
  //std::scoped_lock(jstrajstate_mutex);
  jstrajstate_mutex.lock();

  joint_trajctrl_state_msg = msg;
  ROS_INFO_ONCE("First joint trajectory controller state msg");

  Command command(joint_trajctrl_state_msg);
  latest_command_recd_ = command;

  jstrajstate_mutex.unlock();

}

void RobotMonitor::computeAccelerationandAddtoCommandHistory(Command& latest_command)
{
  //std::vector<double> vel_avg(robot::num_targets, 0.0);
  // if(command_history.size() > 0)
  // {
  //   for(int j = 0; j < robot::num_targets; ++j)
  //   {
  //     double sum = 0.0;
  //     for(int i = 0; i < command_history.size(); ++i)
  //     {
  //       sum = sum + command_history[i].velocity[j];
  //     }
  //     vel_avg[j] = sum/command_history.size();
  //   }
  // }
  // latest_command.velocity_smoothed = vel_avg;

  auto prev_command = command_history.front();


  if(prev_command.absolute_time_stamp < 0.0)
  {
    latest_command.time_diff = cycle_time;
  }
  else
  {
    latest_command.time_diff = cycle_time; //command_history[0].absolute_time_stamp - prev_command.absolute_time_stamp;
  }
  latest_command.relative_time_stamp = prev_command.relative_time_stamp + latest_command.time_diff;

  for(int i = 0; i < robot::num_targets; i++)
  {
    latest_command.acceleration[i] = (latest_command.velocity[i] - prev_command.velocity[i])/latest_command.time_diff;
    latest_command.jerk[i] = (latest_command.acceleration[i] - prev_command.acceleration[i])/latest_command.time_diff;
  }

  command_history.push_front(latest_command);
  if (command_history.size() >= 10)
    command_history.pop_back();
}

const std::deque<Command> RobotMonitor::getCommandHistory()
{
  jstrajstate_mutex.lock();
  Command command = latest_command_recd_;
  jstrajstate_mutex.unlock();

  computeAccelerationandAddtoCommandHistory(command);
  cmd_mutex.lock();
  cmd_a_j = command;
  cmd_mutex.unlock();

  std::deque<Command> cmd_history = command_history;
  return cmd_history;
}

void
RobotMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_INFO_ONCE("RobotMonitor: Received joint state!");
  last_joint_state_mtx.lock();
  last_joint_state_msg = msg;
  last_joint_state_mtx.unlock();

  for (int i=0; i<robot::num_joints; i++)
  { 
    ptrdiff_t pos = find(msg->name.begin(), msg->name.end(), robot::joint_names[i]) - msg->name.begin();
    if(pos < msg->name.size())
    {
      joint_idx_to_msg_idx[i] = pos; // i->joint_idx, pos->msg_idx
    }
    else
    {
      joint_idx_to_msg_idx[i] = -1;
    }
  }
}

void
RobotMonitor::updateLinkThread()
{
  ros::Rate r(100.0); // TODO
  while (ros::ok())
  {
    if (last_joint_state_msg == nullptr){r.sleep(); continue;}

    last_joint_state_mtx.lock();
    sensor_msgs::JointStateConstPtr cached_joint_state_msg = last_joint_state_msg;
    last_joint_state_mtx.unlock();
    
    JointLinkStatePtr new_state = computeJointLinkState(cached_joint_state_msg);
    if (new_state == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    last_joint_link_state_msg = new_state;
    last_joint_link_state_mtx.unlock();
  
    ROS_INFO_ONCE("RobotMonitor: Computed link state!");
    r.sleep();
  }
}

void
RobotMonitor::updateCollisionThread()
{
  ros::Rate r(30.0); // TODO
  while (ros::ok())
  {
    if (last_joint_link_state_msg == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    JointLinkStateConstPtr cached_joint_link_state_msg = last_joint_link_state_msg;
    last_joint_link_state_mtx.unlock();
    
    JointLinkCollisionStatePtr new_state = computeJointLinkCollisionState(cached_joint_link_state_msg);
    if (new_state == nullptr){r.sleep(); continue;}

    last_joint_link_collision_state_mtx.lock();
    last_joint_link_collision_state_msg = new_state;
    last_joint_link_collision_state_mtx.unlock();

    ROS_INFO_ONCE("RobotMonitor: Computed collision state!");
    r.sleep();
  }
}

void
RobotMonitor::updateVisualizationThread()
{
  ros::Rate r(60.0); // 60fps is more than enough!
  while (ros::ok())
  {
    if (last_joint_link_collision_state_msg == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    JointLinkCollisionStateConstPtr cached_joint_link_collision_state_mtx = last_joint_link_collision_state_msg;
    last_joint_link_state_mtx.unlock();
    
    computeAndPublishVisualization(cached_joint_link_collision_state_mtx);

    if (joint_trajctrl_state_msg != nullptr)
    {
      cmd_mutex.lock();
      Command command = cmd_a_j;
      cmd_mutex.unlock();

      sensor_msgs::JointState command_msg;
      command_msg.name.resize(robot::num_targets);
      command_msg.position.resize(robot::num_targets);
      command_msg.velocity.resize(robot::num_targets);
      command_msg.effort.resize(robot::num_targets);

      command_msg.header = joint_trajctrl_state_msg->header;
      command_msg.name = joint_trajctrl_state_msg->joint_names;
      command_msg.position = command.jerk;
      command_msg.velocity = command.velocity;
      command_msg.effort = command.acceleration;

      acc_jerk_pub.publish(command_msg);
    }
    else
    {
      ROS_WARN_THROTTLE(3.0, "Forget to map joint_trajectory_state topic?");
    }


    ROS_INFO_ONCE("RobotMonitor: Published visualization!");
    r.sleep();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

JointLinkStatePtr
RobotMonitor::computeJointLinkState(const sensor_msgs::JointStateConstPtr& msg)
{
  JointLinkStatePtr state = boost::make_shared<JointLinkState>();
  state->joint_state = *msg; // copy
  state->header = msg->header; // copy
  state->header.frame_id = "world"; // TODO

  // recompute kinematic chain
  double variable_positions[robot::num_variables];
  for (int i=0; i<robot::num_variables; i++)
  {
    int joint_idx = robot::variable_idx_to_joint_idx[i];
    int msg_idx = joint_idx_to_msg_idx[joint_idx];
    double joint_val = state->joint_state.position[msg_idx];
    variable_positions[i] = joint_val;
  }
  double global_link_translations[3*robot::num_links];
  double global_link_rotations[4*robot::num_links];
  utils::computeGlobalLinkTransforms((double*)nullptr, (double*)variable_positions, (double*)global_link_translations, (double*)global_link_rotations);

  // Conversion
  std::vector<double> &global_link_transformations = state->link_state.transformations; // copy reference
  global_link_transformations.resize(robot::num_links*7);
  utils::translationRotationToTransform((double*)global_link_translations, (double*)global_link_rotations, global_link_transformations.data(), robot::num_links);

  return state;
}

JointLinkCollisionStatePtr
RobotMonitor::computeJointLinkCollisionState(const JointLinkStateConstPtr& msg)
{
  std::scoped_lock lock_collision_managers(int_collision_manager_mtx, ext_collision_manager_mtx);
  
  JointLinkCollisionStatePtr state = boost::make_shared<JointLinkCollisionState>();
  state->joint_state = msg->joint_state; // copy
  state->link_state = msg->link_state; // copy
  state->header = msg->header; // copy

  // init int_collision_objects if not initialized
  if (int_collision_objects.size() <= 0)
  {
    int_collision_objects = robot::getRobotCollisionObjects(robot::default_inflation);
    for (int i=0; i<int_collision_objects.size(); i++)
    {
      int_collision_objects[i]->setUserData((void*)i);
    }
  }

  // separate transforms to translations and rotations
  double global_link_translations[robot::num_links*3];
  double global_link_rotations[robot::num_links*4];
  utils::transformToTranslationRotation(state->link_state.transformations.data(), 
                                        global_link_translations, 
                                        global_link_rotations, 
                                        state->link_state.transformations.size()/7);

  // compute global object pose
  double global_object_translations[robot::num_objects*3];
  double global_object_rotations[robot::num_objects*4];
  utils::computeCollisionTransforms((const double*)global_link_translations,
                                    (const double*)global_link_rotations,
                                    (const double*)robot::object_transform_translation_only,
                                    (const double*)robot::object_transform_quaternion_only,
                                    (const int*)robot::object_can_skip_translation,
                                    (const int*)robot::object_can_skip_rotation,
                                    (const int*)robot::object_idx_to_link_idx,
                                    robot::num_objects,
                                    (double*)global_object_translations,
                                    (double*)global_object_rotations);

  // update collision manager
  for (int object_idx=0; object_idx<robot::num_objects; object_idx++)
  {
    int_collision_objects[object_idx]->setTranslation(Vec3f(global_object_translations[object_idx*3+0], 
                                                            global_object_translations[object_idx*3+1], 
                                                            global_object_translations[object_idx*3+2]));
    int_collision_objects[object_idx]->setRotation(Eigen::Quaterniond(&(global_object_rotations[object_idx*4])).toRotationMatrix());
    int_collision_objects[object_idx]->computeAABB();
  }
  if (int_collision_manager.size() <= 0) int_collision_manager.registerObjects(int_collision_objects); // init collision manager if not initialized
  int_collision_manager.update();

  // TODO: handle external objects

  // find self collision pairs
  CollisionCallBackCollect self_collision_callback;
  int_collision_manager.collide(&self_collision_callback);
  state->collision_state.int_pair_a.reserve(self_collision_callback.numCollisionPairs()); // uses more memory, but faster
  state->collision_state.int_pair_b.reserve(self_collision_callback.numCollisionPairs()); // uses more memory, but faster
  for (auto &pair : self_collision_callback.getCollisionPairs())
  {
    int first_object_idx = intptr_t(pair.first->getUserData());
    int second_object_idx = intptr_t(pair.second->getUserData());
    int first_link_idx = robot::object_idx_to_link_idx[first_object_idx];
    int second_link_idx = robot::object_idx_to_link_idx[second_object_idx];

    if (first_link_idx == second_link_idx) continue; // skip collisions in the same link
    if (robot::acm[first_link_idx][second_link_idx]) continue; // skip if the collision is allowed

    state->collision_state.int_pair_a.push_back(first_object_idx);
    state->collision_state.int_pair_b.push_back(second_object_idx);
  }

  // TODO: find external collision pairs

  return state;
}

void
RobotMonitor::computeAndPublishVisualization(const JointLinkCollisionStateConstPtr& msg)
{
  if (visualization_pub.getNumSubscribers() <= 0) return;

  visualization_msgs::MarkerArray arr;

  // ========================= Visualize internal collision objects =========================
  int id_counter=0;
  arr.markers.reserve(robot::num_objects);
  for (int i=0; i<robot::num_objects; i++)
  {
    CollisionObject* curr_object = int_collision_objects[i];
    auto curr_translation = curr_object->getTranslation();
    auto curr_rotation = Eigen::Quaterniond(curr_object->getRotation());

    if (curr_object->getNodeType() == hpp::fcl::GEOM_SPHERE)
    {
      const Sphere& shape = static_cast<const Sphere&>(*(curr_object->collisionGeometry()));

      visualization_msgs::Marker marker;
      marker.header = msg->header;
      marker.ns = std::string("robot_collision_body");
      marker.id = id_counter++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = (shape.radius - robot::default_inflation)*2;
      marker.scale.y = (shape.radius - robot::default_inflation)*2;
      marker.scale.z = (shape.radius - robot::default_inflation)*2;
      marker.pose.position.x = curr_translation.x();
      marker.pose.position.y = curr_translation.y();
      marker.pose.position.z = curr_translation.z();
      marker.pose.orientation.x = curr_rotation.x();
      marker.pose.orientation.y = curr_rotation.y();
      marker.pose.orientation.z = curr_rotation.z();
      marker.pose.orientation.w = curr_rotation.w();
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 0.75; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      arr.markers.push_back(marker);
    }
    else if (curr_object->getNodeType() == hpp::fcl::GEOM_CAPSULE)
    {
      ROS_WARN("GEOM_CAPSULE TODO"); // TODO
    }
    else if (curr_object->getNodeType() == hpp::fcl::GEOM_BOX)
    {
      ROS_WARN("GEOM_BOX TODO"); // TODO
    }
    else if (curr_object->getNodeType() == hpp::fcl::GEOM_CYLINDER)
    {
      ROS_WARN("GEOM_CYLINDER TODO"); // TODO
    }
    else if (curr_object->getNodeType() == hpp::fcl::GEOM_PLANE)
    {
      ROS_WARN("GEOM_PLANE TODO"); // TODO
    }
    else
    {
      ROS_ERROR_THROTTLE(1.0, "Unknown collision shape. Can't visualize!");
      continue;
    }
  }

  // ========================= Visualize internal-internal collision pairs =========================
  visualization_msgs::Marker internal_collision_marker;
  internal_collision_marker.header = msg->header;
  internal_collision_marker.ns = std::string("internal_collision");
  internal_collision_marker.id = 0;
  internal_collision_marker.type = visualization_msgs::Marker::LINE_LIST;
  internal_collision_marker.action = visualization_msgs::Marker::ADD;
  internal_collision_marker.color.a = 1.0; // Don't forget to set the alpha!
  internal_collision_marker.color.r = 1.0;
  internal_collision_marker.color.g = 0.0;
  internal_collision_marker.color.b = 0.0;
  internal_collision_marker.pose.orientation.w = 1.0; // bug?
  internal_collision_marker.scale.x = 0.005;
  internal_collision_marker.points.reserve(msg->collision_state.int_pair_a.size()*2);
  for (int i=0; i<msg->collision_state.int_pair_a.size(); i++)
  {
    int object_idx_a = msg->collision_state.int_pair_a[i];
    int object_idx_b = msg->collision_state.int_pair_b[i];
    const auto& translation_a = int_collision_objects[object_idx_a]->getTranslation();
    const auto& translation_b = int_collision_objects[object_idx_b]->getTranslation();

    geometry_msgs::Point point_a;
    point_a.x = translation_a.x();
    point_a.y = translation_a.y();
    point_a.z = translation_a.z();
    internal_collision_marker.points.push_back(point_a);

    geometry_msgs::Point point_b;
    point_b.x = translation_b.x();
    point_b.y = translation_b.y();
    point_b.z = translation_b.z();
    internal_collision_marker.points.push_back(point_b);
  }
  arr.markers.push_back(internal_collision_marker);

  // ========================= Visualize external collision objects =========================

  // ========================= Visualize internal-external collision pairs =========================

  visualization_pub.publish(arr);
}



} // namespace dawn_ik
