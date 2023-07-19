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
const int endpoint_link_idx = 8;
const int num_joints = 18;
const int num_variables = 13;
const int num_links = 18;
const int num_objects = 38;
const int num_acm_link_pairs = 108;
const int num_targets = 6;

// Mapping vectors
const int joint_idx_to_variable_idx[18] = {-1,-1,0,1,2,3,4,5,-1,-1,6,7,8,9,10,11,12,-1}; // -1 if no variable available. Can be used as joint_has_variable vector
const int variable_idx_to_joint_idx[13] = {2,3,4,5,6,7,10,11,12,13,14,15,16};
const int joint_idx_to_target_idx[18] = {-1,-1,0,1,2,3,4,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
const int target_idx_to_joint_idx[6] = {2,3,4,5,6,7};
const int object_idx_to_link_idx[38] = {1,2,2,3,3,3,3,3,3,4,4,4,5,5,5,5,5,6,9,9,9,11,11,11,11,12,12,12,13,13,13,13,14,14,15,15,16,16};

// Joint info
const std::string joint_names[18] = {"world_joint","head_world_joint","head_joint1","head_joint2","head_joint3","head_joint4","head_joint5","head_joint6","head_joint_eef","other_world_joint","other_joint1","other_joint2","other_joint3","other_joint4","other_joint5","other_joint6","other_joint7","other_joint_eef"};
const int joint_child_link_idx[18] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
const int joint_parent_link_idx[18] = {-1,0,1,2,3,4,5,6,7,0,9,10,11,12,13,14,15,16}; // -1 if no link available
const int joint_is_position_bounded[18] = {0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,0}; // bool
const double joint_preferred_position[18] = {0.000000,0.000000,0.000000,0.000000,2.587451,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.017700,0.000000,1.867510,0.000000,0.724311,0.000000,0.000000};
const double joint_max_position[18] = {0.000000,0.000000,6.283185,2.617990,5.235988,6.283185,2.164200,6.283185,0.000000,0.000000,6.283185,2.094400,6.283185,3.927000,6.283185,3.141593,6.283185,0.000000};
const double joint_min_position[18] = {0.000000,0.000000,-6.283185,-2.617990,-0.061087,-6.283185,-2.164200,-6.283185,0.000000,0.000000,-6.283185,-2.059000,-6.283185,-0.191980,-6.283185,-1.692970,-6.283185,0.000000};
const int joint_is_velocity_bounded[18] = {0,0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,0}; // bool
const double joint_max_velocity[18] = {0.000000,0.000000,3.140000,3.140000,3.140000,3.140000,3.140000,3.140000,0.000000,0.000000,3.140000,3.140000,3.140000,3.140000,3.140000,3.140000,3.140000,0.000000};
const double joint_min_velocity[18] = {0.000000,0.000000,-3.140000,-3.140000,-3.140000,-3.140000,-3.140000,-3.140000,0.000000,0.000000,-3.140000,-3.140000,-3.140000,-3.140000,-3.140000,-3.140000,-3.140000,0.000000};
const int joint_is_acceleration_bounded[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // bool
const double joint_max_acceleration[18] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_acceleration[18] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};

// Link info
const std::string link_names[18] = {"ground","head_link_base","head_link1","head_link2","head_link3","head_link4","head_link5","head_link6","head_link_eef","other_link_base","other_link1","other_link2","other_link3","other_link4","other_link5","other_link6","other_link7","other_link_eef"};
const int link_parent_joint_idx[18] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
const double link_transform_translation_only[18][3] = {{0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.243300},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.200000, 0.000000, 0.000000},
                                                       {0.087000, -0.227600, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.061500, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.500000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.267000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, -0.293000, 0.000000},
                                                       {0.052500, 0.000000, 0.000000},
                                                       {0.077500, -0.342500, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.076000, 0.097000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000}};
const double link_transform_quaternion_only[18][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.500004, -0.499998, -0.500002, -0.499996},
                                                      {0.000003, 0.707105, 0.707108, 0.000003},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, -0.707108, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {-0.000000, 0.000000, 0.000000, 1.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707105, -0.707108, 0.000000, 0.000000},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, 0.707108, 0.000000, 0.000000},
                                                      {0.707105, -0.707108, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000}};
const int link_can_skip_translation[18] = {1,1,0,1,0,0,1,0,1,0,0,1,0,0,0,1,0,1}; // bool
const int link_can_skip_rotation[18] = {1,1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,1}; // bool

// ACM
const int acm[18][18]= {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,0,0,0,0,0,1,1,1,1,1,0,0,0,0,1},
                        {1,1,1,1,0,0,0,0,1,1,1,1,1,0,0,0,0,1},
                        {1,0,1,1,1,0,1,1,1,1,1,1,0,0,0,0,0,1},
                        {1,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,1},
                        {1,0,0,0,1,1,1,0,1,0,0,0,0,0,0,0,0,1},
                        {1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1},
                        {1,0,0,1,1,0,1,1,1,0,0,0,0,0,0,0,0,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

// Objective weights
const double weight_preferred_joint_position_goal[18] = {1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000};

// Collision objects info
const double object_transform_translation_only[38][3] = {{0.000000, 0.000000, 0.040000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, -0.075000},
                                                         {0.000000, 0.000000, 0.050000},
                                                         {0.000000, 0.000000, 0.100000},
                                                         {0.065000, 0.000000, 0.100000},
                                                         {0.130000, 0.000000, 0.100000},
                                                         {0.195000, 0.000000, 0.100000},
                                                         {0.195000, 0.000000, 0.050000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.045000, 0.000000, 0.000000},
                                                         {0.090000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, -0.160000},
                                                         {0.000000, 0.000000, -0.100000},
                                                         {0.000000, -0.050000, -0.100000},
                                                         {0.000000, -0.050000, -0.050000},
                                                         {0.000000, -0.050000, 0.000000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, 0.050000},
                                                         {0.000000, 0.000000, 0.150000},
                                                         {0.000000, 0.000000, 0.255000},
                                                         {0.000000, -0.100000, 0.080000},
                                                         {0.000000, 0.000000, 0.080000},
                                                         {0.000000, -0.100000, 0.000000},
                                                         {0.000000, -0.150000, 0.000000},
                                                         {0.052500, 0.030000, 0.000000},
                                                         {0.052500, -0.065000, 0.000000},
                                                         {0.000000, 0.000000, -0.060000},
                                                         {0.080000, -0.090000, 0.000000},
                                                         {0.080000, -0.090000, 0.060000},
                                                         {0.080000, -0.170000, 0.000000},
                                                         {0.040000, -0.045000, 0.070000},
                                                         {0.000000, 0.050000, 0.000000},
                                                         {0.000000, 0.040000, -0.080000},
                                                         {0.000000, 0.000000, 0.020000},
                                                         {0.080000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, 0.100000}};
const double object_transform_quaternion_only[38][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000}};
const int object_can_skip_translation[38] = {0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0}; // bool
const int object_can_skip_rotation[38] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // bool

// Collision Objects Function
const double default_inflation = 0.075;
static inline std::vector<CollisionObject*> getRobotCollisionObjects(double inflation = 0.0)
{
  std::vector<CollisionObject*> objects;

  objects.reserve(38);
  objects.push_back( inflatedCollisionObject(Sphere(0.1), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.06), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.06), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.06), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.04), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.04), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.04), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );

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
