#ifndef __AUTOGENERATED_ROBOT_CONFIGURATION__
#define __AUTOGENERATED_ROBOT_CONFIGURATION__

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <hpp/fcl/shape/geometric_shapes.h>

using namespace ceres;

using hpp::fcl::CollisionObject;
// supported collision shapes:
using hpp::fcl::Box;
using hpp::fcl::Sphere;
using hpp::fcl::Capsule;
using hpp::fcl::Cylinder;
using hpp::fcl::Plane;

namespace robot
{

// Util
template<typename T>
CollisionObject* inflatedCollisionObject(const T &shape, double inflation)
{
  if (inflation <= 0) return new CollisionObject(std::make_shared<T>(shape));
  return new CollisionObject(std::make_shared<T>(shape.inflated(inflation).first));
}

// Constants
const int endpoint_link_idx = 14;
const int num_joints = 17;
const int num_variables = 6;
const int num_links = 17;
const int num_objects = 1;
const int num_acm_link_pairs = 2;
const int num_targets = 6;

// Mapping vectors
const int joint_idx_to_variable_idx[17] = {-1,-1,-1,-1,-1,-1,-1,0,1,2,3,4,5,-1,-1,-1,-1}; // -1 if no variable available. Can be used as joint_has_variable vector
const int variable_idx_to_joint_idx[6] = {7,8,9,10,11,12};
const int joint_idx_to_target_idx[17] = {-1,-1,-1,-1,-1,-1,-1,0,1,2,3,4,5,-1,-1,-1,-1};
const int target_idx_to_joint_idx[6] = {7,8,9,10,11,12};
const int object_idx_to_link_idx[1] = {5};

// Joint info
const std::string joint_names[17] = {"fixed_base","ground_plane_box","world_joint","platform_joint","arm_connection_joint","arm_base_link-base_fixed_joint","arm_base_link-base_link_inertia","arm_shoulder_pan_joint","arm_shoulder_lift_joint","arm_elbow_joint","arm_wrist_1_joint","arm_wrist_2_joint","arm_wrist_3_joint","arm_wrist_3-flange","arm_flange-tool0","arm_tool0_camera-link_fixed_joint","camera-link_camera-depth_fixed_joint"};
const int joint_axis[17] = {0,0,0,0,0,0,0,3,3,3,3,3,3,0,0,0,0};
const int joint_child_link_idx[17] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
const int joint_parent_link_idx[17] = {-1,0,0,2,3,4,4,6,7,8,9,10,11,12,13,14,15}; // -1 if no link available
const int joint_is_position_bounded[17] = {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0}; // bool
const double joint_preferred_position[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_max_position[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,6.283185,6.283185,3.141593,6.283185,6.283185,6.283185,0.000000,0.000000,0.000000,0.000000};
const double joint_min_position[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-6.283185,-6.283185,-3.141593,-6.283185,-6.283185,-6.283185,0.000000,0.000000,0.000000,0.000000};
const int joint_is_velocity_bounded[17] = {0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0}; // bool
const double joint_max_velocity[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,3.141593,3.141593,3.141593,3.141593,3.141593,3.141593,0.000000,0.000000,0.000000,0.000000};
const double joint_min_velocity[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,-3.141593,0.000000,0.000000,0.000000,0.000000};
const int joint_is_acceleration_bounded[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // bool
const double joint_max_acceleration[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_acceleration[17] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};

// Link info
const std::string link_names[17] = {"world","ground_plane_box","platform","base_link","arm_base_link","arm_base","arm_base_link_inertia","arm_shoulder_link","arm_upper_arm_link","arm_forearm_link","arm_wrist_1_link","arm_wrist_2_link","arm_wrist_3_link","arm_flange","arm_tool0","camera_link","camera_depth_frame"};
const int link_parent_joint_idx[17] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
const double link_transform_translation_only[17][3] = {{0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.162500},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {-0.425000, 0.000000, 0.000000},
                                                       {-0.392200, 0.000000, 0.133300},
                                                       {0.000000, -0.099700, -0.000000},
                                                       {0.000000, 0.099600, -0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.020000},
                                                       {0.000000, 0.000000, 0.000000}};
const double link_transform_quaternion_only[17][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.000000, 0.000000, 0.000000, 1.000000},
                                                      {0.000000, 0.000000, 0.000000, 1.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.500000, -0.500000, -0.500000, -0.500000},
                                                      {0.500000, 0.500000, 0.500000, 0.500000},
                                                      {0.500000, -0.500000, -0.500000, -0.500000},
                                                      {0.500000, -0.500000, 0.500000, -0.500000}};
const int link_can_skip_translation[17] = {1,1,0,0,1,1,1,0,1,0,0,0,0,1,1,0,1}; // bool
const int link_can_skip_rotation[17] = {1,0,1,1,1,0,0,1,0,1,1,0,0,0,0,0,0}; // bool

// ACM
const int acm[17][17]= {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

// Objective weights
const double weight_preferred_joint_position_goal[17] = {1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000};

// Collision objects info
const double object_transform_translation_only[1][3] = {{0.000000, 0.000000, 0.040000}};
const double object_transform_quaternion_only[1][4] = {{1.000000, 0.000000, 0.000000, 0.000000}};
const int object_can_skip_translation[1] = {0}; // bool
const int object_can_skip_rotation[1] = {1}; // bool

// Collision Objects Function
const double default_inflation = 0.05;
static inline std::vector<CollisionObject*> getRobotCollisionObjects(double inflation = 0.0)
{
  std::vector<CollisionObject*> objects;

  objects.reserve(1);
  objects.push_back( inflatedCollisionObject(Sphere(0.1), inflation) );

  return objects;
}

// Collision Constraint Function
inline static void setProblemConstraints(ceres::Problem &problem, double *targets_ptr, double *targets_init)
{
}

// Solver Options
static inline void setSolverOptions(ceres::Solver::Options &options)
{
  options.eta = DBL_MIN;
  options.function_tolerance = DBL_MIN;
  options.gradient_tolerance = DBL_MIN;
  options.jacobi_scaling = true;
  options.linear_solver_type = DENSE_QR;
  options.max_num_iterations = 999;
  options.max_solver_time_in_seconds = 0.01;
  options.minimizer_progress_to_stdout = false;
  options.minimizer_type = TRUST_REGION;
  options.parameter_tolerance = DBL_MIN;
}

} // namespace robot

#endif // __AUTOGENERATED_ROBOT_CONFIGURATION__
