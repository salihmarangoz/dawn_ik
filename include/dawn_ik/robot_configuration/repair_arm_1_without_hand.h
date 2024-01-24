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
const int endpoint_link_idx = 29;
const int num_joints = 43;
const int num_variables = 15;
const int num_links = 43;
const int num_objects = 28;
const int num_acm_link_pairs = 130;
const int num_targets = 7;

// Mapping vectors
const int joint_idx_to_variable_idx[43] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,2,3,4,5,6,7,-1,8,9,10,11,12,13,14,-1,-1,-1,-1,-1,-1}; // -1 if no variable available. Can be used as joint_has_variable vector
const int variable_idx_to_joint_idx[15] = {11,22,23,24,25,26,27,28,30,31,32,33,34,35,36};
const int joint_idx_to_target_idx[43] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,1,2,3,4,5,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
const int target_idx_to_joint_idx[7] = {22,23,24,25,26,27,28};
const int object_idx_to_link_idx[28] = {22,22,23,23,24,24,24,25,25,26,26,26,27,28,30,30,31,31,32,32,32,33,33,34,34,34,35,36};

// Joint info
const std::string joint_names[43] = {"ASSUMED_FIXED_ROOT_JOINT","table_top_link_joint","leg1_joint","vbar1_joint","obar1_joint","leg2_joint","vbar2_joint","leg3_joint","vbar3_joint","obar2_joint","sliding_guide_joint","j_torso_1","camera_joint","camera_link_joint","camera_depth_joint","camera_color_joint","camera_color_optical_joint","camera_depth_optical_joint","camera_left_ir_joint","camera_left_ir_optical_joint","camera_right_ir_joint","camera_right_ir_optical_joint","j_arm_1_1","j_arm_1_2","j_arm_1_3","j_arm_1_4","j_arm_1_5","j_arm_1_6","j_arm_1_7","j_tcp_1","j_arm_2_1","j_arm_2_2","j_arm_2_3","j_arm_2_4","j_arm_2_5","j_arm_2_6","j_arm_2_7","j_tcp_2","working_surface_joint","obar3_joint","leg4_joint","vbar4_joint","obar4_joint"};
const int joint_axis[43] = {0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,3,3,-3,3,-3,3,3,0,3,3,-3,3,-3,3,3,0,0,0,0,0,0};
const int joint_child_link_idx[43] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42};
const int joint_parent_link_idx[43] = {-1,0,1,2,3,1,5,1,7,8,9,10,11,12,13,14,15,14,14,18,14,20,11,22,23,24,25,26,27,28,11,30,31,32,33,34,35,36,10,8,1,40,41}; // -1 if no link available
const int joint_is_position_bounded[43] = {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0}; // bool
const double joint_preferred_position[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-1.200000,0.000000,-0.800000,0.000000,0.350000,0.000000,0.000000,0.000000,1.200000,0.000000,0.800000,0.000000,-0.350000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_max_position[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,2.600000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,2.600000,0.100000,2.600000,0.800000,2.800000,2.800000,2.800000,0.000000,2.600000,2.500000,2.600000,2.400000,2.800000,2.100000,2.800000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_position[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-2.600000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-2.600000,-2.500000,-2.600000,-2.400000,-2.800000,-2.100000,-2.800000,0.000000,-2.600000,-0.100000,-2.600000,-0.800000,-2.800000,-2.800000,-2.800000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const int joint_is_velocity_bounded[43] = {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0}; // bool
const double joint_max_velocity[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,11.720000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,3.860000,3.860000,6.060000,6.060000,11.720000,11.720000,11.720000,0.000000,3.860000,3.860000,6.060000,6.060000,11.720000,11.720000,11.720000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_velocity[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-11.720000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-3.860000,-3.860000,-6.060000,-6.060000,-11.720000,-11.720000,-11.720000,0.000000,-3.860000,-3.860000,-6.060000,-6.060000,-11.720000,-11.720000,-11.720000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const int joint_is_acceleration_bounded[43] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // bool
const double joint_max_acceleration[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_acceleration[43] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};

// Link info
const std::string link_names[43] = {"world","table_top_link","leg1_link","vbar1_link","obar1_link","leg2_link","vbar2_link","leg3_link","vbar3_link","obar2_link","sliding_guide_link","torso_1","camera_bottom_screw_frame","camera_link","camera_depth_frame","camera_color_frame","camera_color_optical_frame","camera_depth_optical_frame","camera_left_ir_frame","camera_left_ir_optical_frame","camera_right_ir_frame","camera_right_ir_optical_frame","arm_1_1","arm_1_2","arm_1_3","arm_1_4","arm_1_5","arm_1_6","arm_1_7","arm_1_tcp","arm_2_1","arm_2_2","arm_2_3","arm_2_4","arm_2_5","arm_2_6","arm_2_7","arm_2_tcp","working_surface_link","obar3_link","leg4_link","vbar4_link","obar4_link"};
const int link_parent_joint_idx[43] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42};
const double link_transform_translation_only[43][3] = {{0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.500000, 1.250000, 1.000000},
                                                       {0.000000, 0.000000, 0.485000},
                                                       {-0.500000, 0.000000, 0.485000},
                                                       {-0.500000, 1.250000, 1.000000},
                                                       {0.000000, 0.000000, 0.485000},
                                                       {0.500000, -1.250000, 1.000000},
                                                       {0.000000, 0.000000, 0.485000},
                                                       {-0.500000, 0.000000, 0.485000},
                                                       {0.000000, 1.250000, 0.300000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.085700, 0.000048, -0.273184},
                                                       {0.000000, 0.017500, 0.012500},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.015000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, -0.050000, 0.000000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, 0.135150, -0.242150},
                                                       {-0.060000, 0.030000, 0.125500},
                                                       {0.000000, 0.073000, -0.060150},
                                                       {0.022500, 0.050150, -0.292150},
                                                       {0.000000, 0.072500, -0.050150},
                                                       {0.000000, 0.050000, -0.247500},
                                                       {0.000000, -0.062877, 0.050000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {0.000000, -0.135150, -0.242150},
                                                       {-0.060000, -0.030000, 0.125500},
                                                       {0.000000, -0.073000, -0.060150},
                                                       {0.022500, -0.050150, -0.292150},
                                                       {0.000000, -0.072500, -0.050150},
                                                       {0.000000, -0.050000, -0.247500},
                                                       {0.000000, 0.062877, 0.050000},
                                                       {0.000000, 0.000000, 0.000000},
                                                       {-0.970000, 0.000000, 0.000000},
                                                       {0.000000, 1.250000, 0.485000},
                                                       {-0.500000, -1.250000, 1.000000},
                                                       {0.000000, 0.000000, 0.485000},
                                                       {0.000000, 1.250000, 0.485000}};
const double link_transform_quaternion_only[43][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.000000, -0.707107, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.000000, -0.707107, 0.000000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.500000, 0.500000, 0.500000, 0.500000},
                                                      {0.829918, 0.000406, 0.557886, -0.000273},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.500000, -0.500000, 0.500000, -0.500000},
                                                      {0.500000, -0.500000, 0.500000, -0.500000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.500000, -0.500000, 0.500000, -0.500000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.500000, -0.500000, 0.500000, -0.500000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.707107, 0.000000, -0.707107, 0.000000},
                                                      {-0.500000, -0.500000, -0.500000, 0.500000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {0.697409, 0.697409, 0.116706, -0.116706},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {0.707107, 0.000000, -0.707107, 0.000000},
                                                      {0.500000, -0.500000, 0.500000, 0.500000},
                                                      {0.707107, 0.707107, 0.000000, 0.000000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {0.697409, -0.697409, 0.116706, 0.116706},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {-0.000000, 0.707107, 0.000000, 0.707107},
                                                      {0.707107, -0.707107, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {1.000000, 0.000000, 0.000000, 0.000000},
                                                      {0.707107, -0.707107, 0.000000, 0.000000}};
const int link_can_skip_translation[43] = {1,1,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,1,1,1,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0}; // bool
const int link_can_skip_rotation[43] = {1,1,1,1,0,1,1,1,1,0,0,0,0,1,1,1,0,0,1,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,1,0}; // bool

// ACM
const int acm[43][43]= {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,0,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,1,1,1,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,0,0,1,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,0,1,1,1,1,0,1,1,0},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,0,0,0,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,1,0,0,0,0,0,1,1,1,1,1,1}};

// Objective weights
const double weight_preferred_joint_position_goal[43] = {1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000};

// Collision objects info
const double object_transform_translation_only[28][3] = {{0.000000, 0.000000, 0.025000},
                                                         {0.030000, 0.025000, 0.125000},
                                                         {0.000000, 0.000000, -0.020000},
                                                         {0.000000, 0.085000, -0.060000},
                                                         {0.000000, 0.000000, -0.100000},
                                                         {-0.025000, 0.000000, -0.200000},
                                                         {0.025000, -0.030000, -0.300000},
                                                         {0.000000, 0.000000, -0.025000},
                                                         {0.000000, 0.050000, -0.050000},
                                                         {0.000000, 0.000000, -0.070000},
                                                         {0.000000, 0.070000, -0.150000},
                                                         {0.000000, 0.100000, -0.250000},
                                                         {0.000000, 0.025000, 0.050000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, 0.025000},
                                                         {0.030000, -0.025000, 0.125000},
                                                         {0.000000, 0.000000, -0.020000},
                                                         {0.000000, -0.085000, -0.060000},
                                                         {0.000000, 0.000000, -0.100000},
                                                         {-0.025000, 0.000000, -0.200000},
                                                         {0.025000, 0.030000, -0.300000},
                                                         {0.000000, 0.000000, -0.025000},
                                                         {0.000000, -0.050000, -0.050000},
                                                         {0.000000, 0.000000, -0.070000},
                                                         {0.000000, -0.070000, -0.150000},
                                                         {0.000000, -0.100000, -0.250000},
                                                         {0.000000, -0.025000, 0.050000},
                                                         {0.000000, 0.000000, 0.000000}};
const double object_transform_quaternion_only[28][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
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
const int object_can_skip_translation[28] = {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1}; // bool
const int object_can_skip_rotation[28] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // bool

// Collision Objects Function
const double default_inflation = 0.075;
static inline std::vector<CollisionObject*> getRobotCollisionObjects(double inflation = 0.0)
{
  std::vector<CollisionObject*> objects;

  objects.reserve(28);
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
  objects.push_back( inflatedCollisionObject(Sphere(0.07), inflation) );
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
  objects.push_back( inflatedCollisionObject(Sphere(0.07), inflation) );
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