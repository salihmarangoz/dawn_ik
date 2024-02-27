#ifndef DAWN_IK_GOALS_H
#define DAWN_IK_GOALS_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <dawn_ik/robot_configuration/robot_configuration.h>
#include <dawn_ik/utils.h>
#include <dawn_ik/IKGoal.h>
#include <dawn_ik/JointLinkCollisionState.h>
#include <map>
#include <deque>
#include <vector>

#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#endif

static double acceleration_limit =  M_PI*0.01*0.01*0.5; // TODO

namespace dawn_ik
{

//=================================================================================================
struct SharedBlock
{
  SharedBlock(const dawn_ik::IKGoalPtr &ik_goal,
              std::deque< std::vector<double> >& solver_history,
              std::map<std::string, int> &joint_name_to_joint_idx,
              std::vector<double> &variable_positions,
              std::vector<double> &variable_velocities,
              double (&curr_target_positions)[robot::num_targets],
              double (&curr_target_velocities)[robot::num_targets],
              JointLinkCollisionStateConstPtr &monitor_state,
              const std::vector<CollisionObject*> &int_objects,
              std::deque<Command>& command_history,
              std::vector<double> &filtered_curr_positions
  ):
  ik_goal(ik_goal),
  solver_history(solver_history),
  joint_name_to_joint_idx(joint_name_to_joint_idx),
  variable_positions(variable_positions),
  variable_velocities(variable_velocities),
  curr_target_positions(curr_target_positions),
  curr_target_velocities(curr_target_velocities),
  monitor_state(monitor_state),
  int_objects(int_objects),
  command_history(command_history),
  filtered_curr_positions(filtered_curr_positions)
  {
    // limit m1 endpoint goal distance TODO: MOVE THIS TO THE NODE
    if (ik_goal->m1_limit_dist > 0)
    {
      double xyz_length2 = std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+0] - ik_goal->m1_x, 2)+
                           std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+1] - ik_goal->m1_y, 2)+
                           std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+2] - ik_goal->m1_z, 2);
      if (xyz_length2 < ik_goal->m1_limit_dist*ik_goal->m1_limit_dist)
      {
        m1_x_limited = ik_goal->m1_x;
        m1_y_limited = ik_goal->m1_y;
        m1_z_limited = ik_goal->m1_z;
      }
      else
      {
        double xyz_length = std::sqrt(xyz_length2);
        m1_x_limited = monitor_state->link_state.transformations[7*robot::endpoint_link_idx+0] + 
                                      (ik_goal->m1_limit_dist/xyz_length)*(ik_goal->m1_x - monitor_state->link_state.transformations[7*robot::endpoint_link_idx+0]);
        m1_y_limited = monitor_state->link_state.transformations[7*robot::endpoint_link_idx+1] + 
                                      (ik_goal->m1_limit_dist/xyz_length)*(ik_goal->m1_y - monitor_state->link_state.transformations[7*robot::endpoint_link_idx+1]);
        m1_z_limited = monitor_state->link_state.transformations[7*robot::endpoint_link_idx+2] + 
                                      (ik_goal->m1_limit_dist/xyz_length)*(ik_goal->m1_z - monitor_state->link_state.transformations[7*robot::endpoint_link_idx+2]);
      }
    }
    else
    {
      m1_x_limited = ik_goal->m1_x;
      m1_y_limited = ik_goal->m1_y;
      m1_z_limited = ik_goal->m1_z;
    }

    // TODO: extra
    double xyz_length2_ = std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+0] - ik_goal->m1_x, 2)+
                          std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+1] - ik_goal->m1_y, 2)+
                          std::pow(monitor_state->link_state.transformations[7*robot::endpoint_link_idx+2] - ik_goal->m1_z, 2);
    dist_to_target = sqrt(xyz_length2_);
  }

  const dawn_ik::IKGoalPtr &ik_goal;
  std::deque< std::vector<double> > &solver_history;
  std::map<std::string, int> &joint_name_to_joint_idx;
  std::vector<double> &variable_positions;
  std::vector<double> &variable_velocities;
  double (&curr_target_positions)[robot::num_targets];
  double (&curr_target_velocities)[robot::num_targets];
  JointLinkCollisionStateConstPtr &monitor_state;
  const std::vector<CollisionObject*> &int_objects;
  // modifications
  double m1_x_limited, m1_y_limited, m1_z_limited;
  // extra
  double dist_to_target;

  std::deque<Command> command_history;

  std::vector<double> &filtered_curr_positions;

#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
  moveit::core::RobotModelPtr robot_model_;
  kinematics_metrics::KinematicsMetricsPtr km_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
#endif
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
      // TODO: need to get preferred positions from somewhere
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

struct LimitVelocityGoal{
  LimitVelocityGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {

    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      residuals[target_idx] = T(1.0)*(target_values[target_idx] - shared_block.solver_history[0].at(target_idx));
    }

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<LimitVelocityGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new LimitVelocityGoal(shared_block)));
   }

   SharedBlock &shared_block;
};
//=================================================================================================

/**
 * LimitAccelerationGoal
 * WARNING: This goal may cause collisions or high speed motions
*/
struct LimitAccelerationGoal {
  LimitAccelerationGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      //2 1 0 c -> history
      //c-1 -> current vel with central diff
      //0-2 -> last_vel with central diff
      T current_vel = T(30)*(target_values[target_idx] - shared_block.solver_history[1].at(target_idx));
      double last_vel = 30*(shared_block.solver_history[0].at(target_idx) - shared_block.solver_history[1].at(target_idx));
      residuals[target_idx] = T(30)*(current_vel - T(last_vel));

      // T current_vel = (target_values[target_idx] - shared_block.solver_history[0].at(target_idx)) / 0.01;
      // double last_vel = (shared_block.solver_history[0].at(target_idx) - shared_block.solver_history[1].at(target_idx)) / 0.01 ;
      // residuals[target_idx] = (current_vel - last_vel) / 0.01;


    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<LimitAccelerationGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new LimitAccelerationGoal(shared_block)));
   }

   SharedBlock &shared_block;
};
//=================================================================================================

/**
 * LimitJerkGoal
 * WARNING: This goal may cause collisions or high speed motions
*/
struct LimitJerkGoal {
  LimitJerkGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      //2 1 0 c -> history
      T      v0 = (target_values[target_idx] - shared_block.solver_history[0].at(target_idx));
      double v1 = (shared_block.solver_history[0].at(target_idx) - shared_block.solver_history[1].at(target_idx));
      double v2 = (shared_block.solver_history[1].at(target_idx) - shared_block.solver_history[2].at(target_idx));
      T      a0 = (v0 - v1);
      double a1 = (v1 - v2);
      residuals[target_idx] = T(1000)*(a0-a1);
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<LimitJerkGoal, robot::num_targets, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new LimitJerkGoal(shared_block)));
   }

   SharedBlock &shared_block;
};
//=================================================================================================


/**
 * AvoidJointLimitsGoal
*/
struct AvoidJointLimitsGoal {
  AvoidJointLimitsGoal(SharedBlock &shared_block): shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int target_idx=0; target_idx<robot::num_targets; target_idx++)
    {
      int joint_idx = robot::target_idx_to_joint_idx[target_idx];
      if (robot::joint_is_position_bounded[joint_idx])
      {
        residuals[target_idx*2] = T(0.01) / (target_values[target_idx] - robot::joint_min_position[joint_idx]);
        residuals[target_idx*2+1] = T(0.01) / (target_values[target_idx] - robot::joint_max_position[joint_idx]);
      }
      else
      {
        residuals[target_idx*2] = T(0.0);
        residuals[target_idx*2+1] = T(0.0);
      }
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::AutoDiffCostFunction<AvoidJointLimitsGoal, robot::num_targets*2, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new AvoidJointLimitsGoal(shared_block)));
   }

   SharedBlock &shared_block;
};
//=================================================================================================

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
      //if (shared_block.filtered_curr_positions.size() > 0 )
      //  residuals[i] = target_values[i] - shared_block.filtered_curr_positions[i];
      //else
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
//=================================================================================================

/**
 * EndpointGoal
*/
struct EndpointGoal {
  EndpointGoal(SharedBlock &shared_block) : shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, shared_block.variable_positions.data(), global_link_translations, global_link_rotations);

    // Position cost
    residuals[0] = (global_link_translations[3*robot::endpoint_link_idx+0] - shared_block.m1_x_limited) * shared_block.ik_goal->m1_weight;
    residuals[1] = (global_link_translations[3*robot::endpoint_link_idx+1] - shared_block.m1_y_limited) * shared_block.ik_goal->m1_weight;
    residuals[2] = (global_link_translations[3*robot::endpoint_link_idx+2] - shared_block.m1_z_limited) * shared_block.ik_goal->m1_weight;

    // Orientation cost
    const T qdot =  (global_link_rotations[4*robot::endpoint_link_idx+0] * shared_block.ik_goal->m2_w) + 
                    (global_link_rotations[4*robot::endpoint_link_idx+1] * shared_block.ik_goal->m2_x) + 
                    (global_link_rotations[4*robot::endpoint_link_idx+2] * shared_block.ik_goal->m2_y) + 
                    (global_link_rotations[4*robot::endpoint_link_idx+3] * shared_block.ik_goal->m2_z);

    //residuals[3] = (1.0 - qdot) * shared_block.ik_goal->m2_weight * 20.0;

    //residuals[3] = (1.0 - ceres::abs(qdot)) * shared_block.ik_goal->m2_weight * 20.0;
    
    residuals[3] = shared_block.ik_goal->m2_weight*(global_link_rotations[4*robot::endpoint_link_idx+0] - shared_block.ik_goal->m2_w);
    residuals[4] = shared_block.ik_goal->m2_weight*(global_link_rotations[4*robot::endpoint_link_idx+1] - shared_block.ik_goal->m2_x);
    residuals[5] = shared_block.ik_goal->m2_weight*(global_link_rotations[4*robot::endpoint_link_idx+2] - shared_block.ik_goal->m2_y);
    residuals[6] = shared_block.ik_goal->m2_weight*(global_link_rotations[4*robot::endpoint_link_idx+3] - shared_block.ik_goal->m2_z);
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     //return (new ceres::NumericDiffCostFunction<EndpointGoal, ceres::FORWARD, 4, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new EndpointGoal(shared_block)));
     return (new ceres::AutoDiffCostFunction<EndpointGoal, 7, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new EndpointGoal(shared_block)));
   }

  SharedBlock &shared_block;
};
//=================================================================================================

/**
 * LookAtGoal
*/
struct LookAtGoal {
  LookAtGoal(SharedBlock &shared_block) : shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, shared_block.variable_positions.data(), global_link_translations, global_link_rotations);

    // target position w.r.t. eef position (world orientation)
    T target_x = shared_block.ik_goal->m3_x - global_link_translations[3*robot::endpoint_link_idx+0];
    T target_y = shared_block.ik_goal->m3_y - global_link_translations[3*robot::endpoint_link_idx+1];
    T target_z = shared_block.ik_goal->m3_z - global_link_translations[3*robot::endpoint_link_idx+2];

    // currently looked position w.r.t. eef position (world orientation)
    T q[4];
    q[0] = global_link_rotations[4*robot::endpoint_link_idx+0];
    q[1] = global_link_rotations[4*robot::endpoint_link_idx+1];
    q[2] = global_link_rotations[4*robot::endpoint_link_idx+2];
    q[3] = global_link_rotations[4*robot::endpoint_link_idx+3];
    T fv[3];
    fv[0] = T(0.0);
    fv[1] = T(0.0);
    fv[2] = T(1.0);
    T result[3];
    ceres::UnitQuaternionRotatePoint(q, fv, result);
    T eef_x = result[0];
    T eef_y = result[1];
    T eef_z = result[2];

    // maybe cosine similarity...
    T target_norm = ceres::sqrt(target_x*target_x + target_y*target_y + target_z*target_z);
    T eef_norm = ceres::sqrt(eef_x*eef_x + eef_y*eef_y + eef_z*eef_z);
    //residuals[0] = shared_block.ik_goal->m3_weight * ceres::acos((target_x*eef_x + target_y*eef_y + target_z*eef_z)/(target_norm*eef_norm));
    residuals[0] = 10.0*shared_block.ik_goal->m3_weight * (target_x/target_norm - eef_x);
    residuals[1] = 10.0*shared_block.ik_goal->m3_weight * (target_y/target_norm - eef_y);
    residuals[2] = 10.0*shared_block.ik_goal->m3_weight * (target_z/target_norm - eef_z);

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     //return (new ceres::NumericDiffCostFunction<LookAtGoal, ceres::FORWARD, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new LookAtGoal(shared_block)));
     return (new ceres::AutoDiffCostFunction<LookAtGoal, 3, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new LookAtGoal(shared_block)));
   }

  SharedBlock &shared_block;
};
//=================================================================================================

/**
 * DirectionGoal
*/
struct DirectionGoal {
  DirectionGoal(SharedBlock &shared_block) : shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, shared_block.variable_positions.data(), global_link_translations, global_link_rotations);

    // currently looked position w.r.t. eef position (world orientation)
    T q[4];
    q[0] = global_link_rotations[4*robot::endpoint_link_idx+0];
    q[1] = global_link_rotations[4*robot::endpoint_link_idx+1];
    q[2] = global_link_rotations[4*robot::endpoint_link_idx+2];
    q[3] = global_link_rotations[4*robot::endpoint_link_idx+3];
    T fv[3];
    fv[0] = T(0.0);
    fv[1] = T(0.0);
    fv[2] = T(1.0);
    T result[3];
    ceres::UnitQuaternionRotatePoint(q, fv, result);
    T eef_x = result[0];
    T eef_y = result[1];
    T eef_z = result[2];

    residuals[0] = 10.0*shared_block.ik_goal->m4_weight * (shared_block.ik_goal->m4_x - eef_x);
    residuals[1] = 10.0*shared_block.ik_goal->m4_weight * (shared_block.ik_goal->m4_y - eef_y);
    residuals[2] = 10.0*shared_block.ik_goal->m4_weight * (shared_block.ik_goal->m4_z - eef_z);

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     //return (new ceres::NumericDiffCostFunction<DirectionGoal, ceres::FORWARD, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new DirectionGoal(shared_block)));
     return (new ceres::AutoDiffCostFunction<DirectionGoal, 3, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new DirectionGoal(shared_block)));
   }

  SharedBlock &shared_block;
};
//=================================================================================================

/**
 * DistanceToGoal
*/
struct DistanceToGoal {
  DistanceToGoal(SharedBlock &shared_block) : shared_block(shared_block) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];
    utils::computeGlobalLinkTransforms(target_values, shared_block.variable_positions.data(), global_link_translations, global_link_rotations);

    // target position w.r.t. eef position (world orientation)
    T target_x = shared_block.ik_goal->m3_x - global_link_translations[3*robot::endpoint_link_idx+0];
    T target_y = shared_block.ik_goal->m3_y - global_link_translations[3*robot::endpoint_link_idx+1];
    T target_z = shared_block.ik_goal->m3_z - global_link_translations[3*robot::endpoint_link_idx+2];
    T dist = ceres::sqrt(target_x*target_x + target_y*target_y + target_z*target_z);

    // Maybe add a growing term to prevent the optimizer to exclude this objective?
    T steep = T(50.0);
    T max_dist = T(0.4);
    T min_dist = T(0.1);
    residuals[0] = ceres::tanh(steep*(dist-max_dist)) + ceres::tanh(steep*(-dist+min_dist)) + 2.0;

    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     //return (new ceres::NumericDiffCostFunction<DistanceToGoal, ceres::FORWARD, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new DistanceToGoal(shared_block)));
     return (new ceres::AutoDiffCostFunction<DistanceToGoal, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new DistanceToGoal(shared_block)));
   }

  SharedBlock &shared_block;
};
//=================================================================================================

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
      const T distance = utils::distSphere2Sphere(pos_a, 
                                          shape_a.radius-robot::default_inflation, 
                                          pos_b, 
                                          shape_b.radius-robot::default_inflation);
      double weight = 0.005;
      double eps = 0.00001;
      if (distance <= 0)
        residuals[i] = T(weight/eps);
      else// if (distance < 0.1)
        residuals[i] = weight / (distance+eps);
      //else
      //  residuals[i] = T(0.0);
    }

    return true; 
  }

  
  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(SharedBlock &shared_block)
  {
    int num_int_pairs = shared_block.monitor_state->collision_state.int_pair_a.size();
    //return (new ceres::NumericDiffCostFunction<CollisionAvoidanceGoal, ceres::FORWARD, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
    //            new CollisionAvoidanceGoal(shared_block), ceres::TAKE_OWNERSHIP, num_int_pairs));
    return (new ceres::AutoDiffCostFunction<CollisionAvoidanceGoal, ceres::DYNAMIC, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                new CollisionAvoidanceGoal(shared_block), num_int_pairs));

  }

  SharedBlock &shared_block;
};

// ==================================================================

/**
 * CollisionAvoidanceGoalNumeric
*/
struct CollisionAvoidanceGoalNumeric {
  CollisionAvoidanceGoalNumeric(SharedBlock &shared_block): shared_block(shared_block) {}

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

    int num_int_pairs = shared_block.monitor_state->collision_state.int_pair_a.size();
    for (int i=0; i<num_int_pairs; i++)
    {
      int object_idx_a = shared_block.monitor_state->collision_state.int_pair_a[i];
      int object_idx_b = shared_block.monitor_state->collision_state.int_pair_b[i];
      int link_idx_a = robot::object_idx_to_link_idx[object_idx_a];
      int link_idx_b = robot::object_idx_to_link_idx[object_idx_b];
      const CollisionObject* object_a = shared_block.int_objects[object_idx_a];
      const CollisionObject* object_b = shared_block.int_objects[object_idx_b];

      //const Sphere& shape_a = static_cast<const Sphere&>(*(object_a->collisionGeometry()));
      //const Sphere& shape_b = static_cast<const Sphere&>(*(object_b->collisionGeometry()));
      const T* pos_a = &(global_object_translations[object_idx_a*3]);
      const T* pos_b = &(global_object_translations[object_idx_b*3]);

      hpp::fcl::Transform3f T1;
      T1.setQuatRotation(hpp::fcl::Quaternion3f(global_object_rotations[object_idx_a*4+0],
                                                global_object_rotations[object_idx_a*4+1],
                                                global_object_rotations[object_idx_a*4+2],
                                                global_object_rotations[object_idx_a*4+3]));
      T1.setTranslation(hpp::fcl::Vec3f(global_object_translations[object_idx_a*3+0],
                                        global_object_translations[object_idx_a*3+1],
                                        global_object_translations[object_idx_a*3+2]));

      hpp::fcl::Transform3f T2;
      T2.setQuatRotation(hpp::fcl::Quaternion3f(global_object_rotations[object_idx_b*4+0],
                                                global_object_rotations[object_idx_b*4+1],
                                                global_object_rotations[object_idx_b*4+2],
                                                global_object_rotations[object_idx_b*4+3]));
      T2.setTranslation(hpp::fcl::Vec3f(global_object_translations[object_idx_b*3+0],
                                        global_object_translations[object_idx_b*3+1],
                                        global_object_translations[object_idx_b*3+2]));

      hpp::fcl::DistanceRequest dist_req;
      hpp::fcl::DistanceResult dist_res;
      hpp::fcl::distance(object_a->collisionGeometry().get(), T1, object_b->collisionGeometry().get(), T2, dist_req, dist_res);

      const T distance = dist_res.min_distance;

      double weight = 0.005;
      double eps = 0.00001;
      if (distance <= 0)
        residuals[i] = T(weight/eps);
      else// if (distance < 0.1)
        residuals[i] = weight / (distance+eps);
      //else
      //  residuals[i] = T(0.0);
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



// ======================================================================




#ifdef ENABLE_EXPERIMENT_MANIPULABILITY
/**
 * ManipulabilityGoal
*/
struct ManipulabilityGoal {
  ManipulabilityGoal(SharedBlock &shared_block) : shared_block(shared_block)
  {

  }

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    // Create a robot state using the joint values
    moveit::core::RobotState robot_state(shared_block.robot_model_);
    for(size_t variable_idx = 0; variable_idx < robot::num_variables; variable_idx++)
    {
      int joint_idx = robot::variable_idx_to_joint_idx[variable_idx];
      int target_idx = robot::joint_idx_to_target_idx[joint_idx];

      if (target_idx < 0)
      {
        robot_state.setVariablePosition(variable_idx, shared_block.variable_positions[variable_idx]);
      }
      else
      {
        robot_state.setVariablePosition(variable_idx, target_values[target_idx]);
      }
    }
    robot_state.updateLinkTransforms();

    // double manipulability_index_tra;
    // shared_block.km_->getManipulabilityIndex(robot_state, "lite6", manipulability_index_tra, true);
    // ROS_INFO_THROTTLE(1.0, "manipulability_index_tra: %f", manipulability_index_tra);
    // if (std::isnan(manipulability_index_tra)) return false; // sad jacobian sounds
    // residuals[0] = 1.0 - manipulability_index_tra;

    double manipulability_index_rot;
    shared_block.km_->getManipulabilityIndex(robot_state, "lite6", manipulability_index_rot, false);
    ROS_INFO_THROTTLE(1.0, "manipulability_index_rot: %f", manipulability_index_rot);
    if (std::isnan(manipulability_index_rot)) return false; // sad jacobian sounds
    residuals[0] = 1.0 - manipulability_index_rot;
    
    return true;
  }

   // Factory to hide the construction of the CostFunction object from the client code.
   static ceres::CostFunction* Create(SharedBlock &shared_block)
   {
     return (new ceres::NumericDiffCostFunction<ManipulabilityGoal, ceres::FORWARD, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new ManipulabilityGoal(shared_block)));
     //return (new ceres::AutoDiffCostFunction<ManipulabilityGoal, 1, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
     //            new ManipulabilityGoal(shared_block)));
   }

  SharedBlock &shared_block;
};
#endif


} // namespace dawn_ik

#endif // DAWN_IK_GOALS_H
