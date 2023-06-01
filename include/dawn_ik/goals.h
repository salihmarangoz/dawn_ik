#ifndef DAWN_IK_GOALS_H
#define DAWN_IK_GOALS_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <dawn_ik/utils.h>
#include <dawn_ik/IKGoal.h>
#include <dawn_ik/JointLinkCollisionState.h>
#include <map>
#include <queue>
#include <vector>

namespace dawn_ik
{

//=================================================================================================
struct SharedBlock
{
  SharedBlock(const dawn_ik::IKGoalPtr &ik_goal,
              std::queue< std::vector<double> > solver_history,
              std::map<std::string, int> &joint_name_to_joint_idx,
              std::vector<double> &variable_positions,
              std::vector<double> &variable_velocities,
              double (&curr_target_positions)[robot::num_targets],
              double (&curr_target_velocities)[robot::num_targets],
              JointLinkCollisionStateConstPtr &monitor_state,
              const std::vector<CollisionObject*> &int_objects
  ):
  ik_goal(ik_goal),
  solver_history(solver_history),
  joint_name_to_joint_idx(joint_name_to_joint_idx),
  variable_positions(variable_positions),
  variable_velocities(variable_velocities),
  curr_target_positions(curr_target_positions),
  curr_target_velocities(curr_target_velocities),
  monitor_state(monitor_state),
  int_objects(int_objects)
  {};
  const dawn_ik::IKGoalPtr &ik_goal;
  std::queue< std::vector<double> > &solver_history;
  std::map<std::string, int> &joint_name_to_joint_idx;
  std::vector<double> &variable_positions;
  std::vector<double> &variable_velocities;
  double (&curr_target_positions)[robot::num_targets];
  double (&curr_target_velocities)[robot::num_targets];
  JointLinkCollisionStateConstPtr &monitor_state;
  const std::vector<CollisionObject*> &int_objects;
};
//=================================================================================================

/**
 * PreferredJointPositionGoal
*/
struct PreferredJointPositionGoal {
  PreferredJointPositionGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      int joint_idx = robot::target_idx_to_joint_idx[target_idx];

      // TODO: temporarly disable this feature for unbounded joints
      // if (!robot::joint_is_position_bounded[joint_idx])
      // {
      //   residuals[target_idx] = T(0.0);
      // }

      residuals[target_idx] = robot::weight_preferred_joint_position_goal[joint_idx] * (target_values[target_idx] - robot::joint_preferred_position[joint_idx]);
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<PreferredJointPositionGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new PreferredJointPositionGoal(shared_block)));
   }

   SharedBlock &shared_block;
};

/**
 * MinimalJointDisplacementGoal
*/
struct MinimalJointDisplacementGoal {
  MinimalJointDisplacementGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets; i++)
    {
      residuals[i] = target_values[i] - shared_block.curr_target_positions[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<MinimalJointDisplacementGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new MinimalJointDisplacementGoal(shared_block)));
   }
   
   SharedBlock &shared_block;
};

/**
 * EndpointGoal
*/
struct EndpointGoal {
  EndpointGoal(SharedBlock &shared_block, const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction) : shared_block(shared_block), endpoint(endpoint), direction(direction) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, shared_block.variable_positions.data(), global_link_translations, global_link_rotations);

    // Position cost
    residuals[0] = ceres::hypot(global_link_translations[3*robot::endpoint_link_idx+0] - endpoint[0],
                                global_link_translations[3*robot::endpoint_link_idx+1] - endpoint[1],
                                global_link_translations[3*robot::endpoint_link_idx+2] - endpoint[2]);

    // Orientation cost
    residuals[1] = global_link_rotations[4*robot::endpoint_link_idx+0] - direction.w();
    residuals[2] = global_link_rotations[4*robot::endpoint_link_idx+1] - direction.x();
    residuals[3] = global_link_rotations[4*robot::endpoint_link_idx+2] - direction.y();
    residuals[4] = global_link_rotations[4*robot::endpoint_link_idx+3] - direction.z();
    
    // TEST: SLOW!
    /*
    Eigen::Quaterniond endpoint_dir;
    endpoint_dir.w() = global_link_rotations[4*robot::endpoint_link_idx+0];
    endpoint_dir.x() = global_link_rotations[4*robot::endpoint_link_idx+1];
    endpoint_dir.y() = global_link_rotations[4*robot::endpoint_link_idx+2];
    endpoint_dir.z() = global_link_rotations[4*robot::endpoint_link_idx+3];
    residuals[1] = endpoint_dir.angularDistance(direction);
    */

    // TEST: SLOW!
    /*
    residuals[1] = 1.0-global_link_rotations[4*robot::endpoint_link_idx+0]*direction.w() -
                   global_link_rotations[4*robot::endpoint_link_idx+1]*direction.x() -
                   global_link_rotations[4*robot::endpoint_link_idx+2]*direction.y() -
                   global_link_rotations[4*robot::endpoint_link_idx+3]*direction.z();
    */

    // couldn't implement this method
    /*
    T inverse_quad[4];
    inverse_quad[0] = global_link_rotations[4*robot::endpoint_link_idx+0];
    inverse_quad[1] = -global_link_rotations[4*robot::endpoint_link_idx+1];
    inverse_quad[2] = -global_link_rotations[4*robot::endpoint_link_idx+2];
    inverse_quad[3] = -global_link_rotations[4*robot::endpoint_link_idx+3];
    T endpoint[4];
    endpoint[0] = direction.w();
    endpoint[1] = direction.x();
    endpoint[2] = direction.y();
    endpoint[3] = direction.z();
    T tmp[4];
    ceres::QuaternionProduct(endpoint, inverse_quad, tmp);
    */

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block, const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction)
   {
     //return (new ceres::NumericDiffCostFunction<EndpointGoal, ceres::FORWARD, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new EndpointGoal(shared_block, endpoint, direction)));
     return (new ceres::AutoDiffCostFunction<EndpointGoal, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new EndpointGoal(shared_block, endpoint, direction)));
   }

  SharedBlock &shared_block;
  const Eigen::Vector3d endpoint;       // TODO
  const Eigen::Quaterniond direction;   // TODO
};

/**
 * CollisionAvoidanceGoal
*/
struct CollisionAvoidanceGoal {
  CollisionAvoidanceGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    // Jet conversion is needed for some variables because we have unknown number of residuals in this structure
    T variable_positions_JET[robot::num_variables];
    for (int i=0; i<robot::num_variables; i++) variable_positions_JET[i] = T(shared_block.variable_positions[i]);
    T object_transform_translation_only_JET[robot::num_objects*3]; 
    T object_transform_quaternion_only_JET[robot::num_objects*4]; 
    for (int i=0; i<robot::num_objects; i++)
    {
      object_transform_translation_only_JET[i*3+0] = T(robot::object_transform_translation_only[i][0]);
      object_transform_translation_only_JET[i*3+1] = T(robot::object_transform_translation_only[i][1]);
      object_transform_translation_only_JET[i*3+2] = T(robot::object_transform_translation_only[i][2]);
      object_transform_quaternion_only_JET[i*4+0] = T(robot::object_transform_quaternion_only[i][0]);
      object_transform_quaternion_only_JET[i*4+1] = T(robot::object_transform_quaternion_only[i][1]);
      object_transform_quaternion_only_JET[i*4+2] = T(robot::object_transform_quaternion_only[i][2]);
      object_transform_quaternion_only_JET[i*4+3] = T(robot::object_transform_quaternion_only[i][3]);
    }

    // Compute link transforms
    T global_link_translations[robot::num_links*3];
    T global_link_rotations[robot::num_links*4];
    utils::computeGlobalLinkTransforms(target_values, variable_positions_JET, global_link_translations, global_link_rotations);

    // Compute internal object positions
    T global_object_translations[robot::num_objects*3];
    T global_object_rotations[robot::num_objects*4];
    utils::computeCollisionTransforms((const T*)global_link_translations,
                                      (const T*)global_link_rotations,
                                      (const T*)object_transform_translation_only_JET,
                                      (const T*)object_transform_quaternion_only_JET,
                                      (const int*)robot::object_can_skip_translation,
                                      (const int*)robot::object_can_skip_rotation,
                                      (const int*)robot::object_idx_to_link_idx,
                                      robot::num_objects,
                                      (T*)global_object_translations,
                                      (T*)global_object_rotations);

    // TODO: supports only spheres!!!
    int num_int_pairs = shared_block.monitor_state->collision_state.int_pair_a.size();
    for (int i=0; i<num_int_pairs; i++)
    {
      int object_idx_a = shared_block.monitor_state->collision_state.int_pair_a[i];
      int object_idx_b = shared_block.monitor_state->collision_state.int_pair_b[i];
      int link_idx_a = robot::object_idx_to_link_idx[object_idx_a];
      int link_idx_b = robot::object_idx_to_link_idx[object_idx_b];
      const CollisionObject* object_a = shared_block.int_objects[object_idx_a];
      const CollisionObject* object_b = shared_block.int_objects[object_idx_b];
      const Sphere& shape_a = static_cast<const Sphere&>(*(object_a->collisionGeometry()));
      const Sphere& shape_b = static_cast<const Sphere&>(*(object_b->collisionGeometry()));
      const T* pos_a = &(global_object_translations[object_idx_a*3]);
      const T* pos_b = &(global_object_translations[object_idx_b*3]);

      // TODO: https://github.com/humanoid-path-planner/hpp-fcl/blob/devel/test/box_box_distance.cpp

      // TODO: write a function for this!
      // TODO: maybe use not inflated objects? inflated ones only for the broadphase collision detection.
      const T distance = utils::distSphere2Sphere(pos_a, 
                                                  shape_a.radius-robot::default_inflation, 
                                                  pos_b, 
                                                  shape_b.radius-robot::default_inflation);

      // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if (distance < 0.05)
        residuals[i] = 0.001 / (distance);
      else
        residuals[i] = 0.0;

      
      if (distance < 0) return false; // TODO: maybe return false if there is a collision?
    }

    return true; 
  }

  
  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(SharedBlock &shared_block)
  {
    int num_int_pairs = shared_block.monitor_state->collision_state.int_pair_a.size();
    return (new ceres::NumericDiffCostFunction<CollisionAvoidanceGoal, ceres::FORWARD, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                new CollisionAvoidanceGoal(shared_block), ceres::TAKE_OWNERSHIP, num_int_pairs));
    //return (new ceres::AutoDiffCostFunction<CollisionAvoidanceGoal, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
    //            new CollisionAvoidanceGoal(shared_block), num_int_pairs));

  }

  SharedBlock &shared_block;
};







} // namespace dawn_ik

#endif // DAWN_IK_GOALS_H