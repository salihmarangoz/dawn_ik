#ifndef __CERES_IK_H__
#define __CERES_IK_H__

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <salih_marangoz_thesis/utils/ceres_utils.h>
#include <salih_marangoz_thesis/utils/moveit_utils.h>

#include <visualization_msgs/InteractiveMarkerFeedback.h> // TODO

namespace salih_marangoz_thesis
{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class CeresIK
{
public:
  std::random_device rand_dev;
  std::mt19937 rand_gen;
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;

  // TODO
  void subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg);
  ros::Subscriber endpoint_sub; 
  Eigen::Vector3d endpoint;
  Eigen::Quaterniond direction;

  ros::Publisher vis_pub;

  ros::Publisher marker_array_pub;

  CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh);
  moveit::core::RobotState getCurrentRobotState();
  void loop();
  bool update(moveit::core::RobotState &current_state);

};

/**
 * MinimalJointDisplacementGoal
*/
struct MinimalJointDisplacementGoal {
  MinimalJointDisplacementGoal(const double *init_target_positions)
  :init_target_positions(init_target_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets-2; i++) // TODO: skip the last joint. actually this should be modifiable in the robot configuration
    {
      residuals[i] = target_values[i] - init_target_positions[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double *init_target_positions)
   { // TODO: robot::num_targets-2 !!!!!!!!!!!!!!
     return (new ceres::AutoDiffCostFunction<MinimalJointDisplacementGoal, robot::num_targets-2, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new MinimalJointDisplacementGoal(init_target_positions)));
   }
   
   const double* init_target_positions;
};


/**
 * CenterJointsGoal
*/
struct CenterJointsGoal {
  CenterJointsGoal(const double* target_centers)
  : target_centers(target_centers){}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    for (int i=0; i<robot::num_targets-2; i++) // TODO: skip the last joint. actually this should be modifiable in the robot configuration
    {
      residuals[i] = target_values[i] - target_centers[i];
    }
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const double* target_centers)
   { // TODO: robot::num_targets-2 !!!!!!!!!!!!!!
     return (new ceres::AutoDiffCostFunction<CenterJointsGoal, robot::num_targets-2, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new CenterJointsGoal(target_centers)));
   }
   
   const double* target_centers;
};

struct EndpointGoal {
  EndpointGoal(const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction, const int (&joint_idx_to_target_idx)[robot::num_joints], const double* variable_positions) : endpoint(endpoint), direction(direction), joint_idx_to_target_idx(joint_idx_to_target_idx), variable_positions(variable_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];

    // TODO: convert to function
    T link_translations[3*robot::num_links];
    T link_rotations[4*robot::num_links];
    for (int i=0; i<robot::num_links; i++)
    {
      link_translations[3*i+0] = T(robot::link_transform_translation_only[i][0]);
      link_translations[3*i+1] = T(robot::link_transform_translation_only[i][1]);
      link_translations[3*i+2] = T(robot::link_transform_translation_only[i][2]);
      link_rotations[4*i+0] = T(robot::link_transform_quaternion_only[i][0]);
      link_rotations[4*i+1] = T(robot::link_transform_quaternion_only[i][1]);
      link_rotations[4*i+2] = T(robot::link_transform_quaternion_only[i][2]);
      link_rotations[4*i+3] = T(robot::link_transform_quaternion_only[i][3]);
    }

    for (int i=0; i<robot::num_joints; i++)
    {
      int child_link_idx = robot::joint_child_link_idx[i];
      int parent_link_idx = robot::joint_parent_link_idx[i];
      int target_idx = joint_idx_to_target_idx[i];
      int variable_idx = robot::joint_idx_to_variable_idx[i];

      // init
      if (parent_link_idx == -1)
      {
        // TODO: convert to function
        global_link_translations[3*child_link_idx+0] = T(0.0);
        global_link_translations[3*child_link_idx+1] = T(0.0);
        global_link_translations[3*child_link_idx+2] = T(0.0);
        global_link_rotations[4*child_link_idx+0] = T(1.0);
        global_link_rotations[4*child_link_idx+1] = T(0.0);
        global_link_rotations[4*child_link_idx+2] = T(0.0);
        global_link_rotations[4*child_link_idx+3] = T(0.0);
        continue;
      }

      // Translation
      if (robot::link_can_skip_translation[child_link_idx])
      {
          // TODO: convert to function
          global_link_translations[3*child_link_idx+0] = global_link_translations[3*parent_link_idx+0];
          global_link_translations[3*child_link_idx+1] = global_link_translations[3*parent_link_idx+1];
          global_link_translations[3*child_link_idx+2] = global_link_translations[3*parent_link_idx+2];
      }
      else
      {
        utils::computeLinkTranslation(&(global_link_translations[3*parent_link_idx]), 
                                      &(global_link_rotations[4*parent_link_idx]), 
                                      &(link_translations[3*child_link_idx]), 
                                      &(global_link_translations[3*child_link_idx]));
      }

      if (variable_idx!=-1) // if joint can move
      { 
        T joint_val;
        if (target_idx!=-1)
        { // this is an optimization target
          joint_val = target_values[target_idx];
        }
        else
        { // this is a joint value but not an optimization target
          joint_val = T(variable_positions[variable_idx]);
        }

        if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then only rotate using the joint position
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]),
                                    joint_val, 
                                    &(global_link_rotations[4*child_link_idx]));
        }
        else // if link has rotation and joint has rotation, then we need to rotate using both
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                    &(link_rotations[4*child_link_idx]), 
                                    joint_val, 
                                    &(global_link_rotations[4*child_link_idx]));
        }
      }
      else // if joint is static
      {
        if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then no need to do anything
        {
          // TODO: convert to function
          global_link_rotations[4*child_link_idx+0] = global_link_rotations[4*parent_link_idx+0];
          global_link_rotations[4*child_link_idx+1] = global_link_rotations[4*parent_link_idx+1];
          global_link_rotations[4*child_link_idx+2] = global_link_rotations[4*parent_link_idx+2];
          global_link_rotations[4*child_link_idx+3] = global_link_rotations[4*parent_link_idx+3];
        }
        else // if link has a rotation, only compute that
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                    &(link_rotations[4*child_link_idx]),
                                    &(global_link_rotations[4*child_link_idx]));
        }

      }
    }

    // Simpler, maybe faster?
    //residuals[0] = global_link_translations[3*robot::endpoint_link_idx+0] - endpoint[0];
    //residuals[1] = global_link_translations[3*robot::endpoint_link_idx+1] - endpoint[1];
    //residuals[2] = global_link_translations[3*robot::endpoint_link_idx+2] - endpoint[2];

    // L2 distance to the endpoint. Seems correct
    residuals[0] = ceres::hypot(global_link_translations[3*robot::endpoint_link_idx+0] - endpoint[0],
                                global_link_translations[3*robot::endpoint_link_idx+1] - endpoint[1],
                                global_link_translations[3*robot::endpoint_link_idx+2] - endpoint[2]);

    // angle diff: BAD
    //residuals[1] = ceres::acos(global_link_rotations[4*robot::endpoint_link_idx+0]*direction.w()+
    //                           global_link_rotations[4*robot::endpoint_link_idx+1]*direction.x()+
    //                           global_link_rotations[4*robot::endpoint_link_idx+2]*direction.y()+
    //                           global_link_rotations[4*robot::endpoint_link_idx+3]*direction.z());

    // element-wise diff: GOOD
    residuals[1] = global_link_rotations[4*robot::endpoint_link_idx+0] - direction.w();
    residuals[2] = global_link_rotations[4*robot::endpoint_link_idx+1] - direction.x();
    residuals[3] = global_link_rotations[4*robot::endpoint_link_idx+2] - direction.y();
    residuals[4] = global_link_rotations[4*robot::endpoint_link_idx+3] - direction.z();

    // quaternion product -> 1-w as a cost? WEIRD
    //T direction_[4];
    //direction_[0] = T(direction.w());
    //direction_[1] = T(direction.x());
    //direction_[2] = T(direction.y());
    //direction_[3] = T(direction.z());
    //T result[4];
    //ceres::QuaternionProduct(direction_, &(global_link_rotations[4*robot::endpoint_link_idx]), result);
    //residuals[1] = 1.0 - result[0];

    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const Eigen::Vector3d &endpoint, const Eigen::Quaterniond &direction, const int (&joint_idx_to_target_idx)[robot::num_joints], const double* variable_positions)
   {
     return (new ceres::AutoDiffCostFunction<EndpointGoal, 5, robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new EndpointGoal(endpoint, direction, joint_idx_to_target_idx, variable_positions)));
   }

  const Eigen::Vector3d endpoint;
  const Eigen::Quaterniond direction;
  const int (&joint_idx_to_target_idx)[robot::num_joints];
  const double* variable_positions;
};


struct CollisionAvoidanceGoal {
  CollisionAvoidanceGoal(const int (&joint_idx_to_target_idx)[robot::num_joints], const double* variable_positions) : joint_idx_to_target_idx(joint_idx_to_target_idx), variable_positions(variable_positions) {}

  template <typename T>
  bool operator()(const T* target_values, T* residuals) const // param_x, param_y, residuals
  {
    T global_link_translations[3*robot::num_links];
    T global_link_rotations[4*robot::num_links];

    // TODO: convert to function
    T link_translations[3*robot::num_links];
    T link_rotations[4*robot::num_links];
    for (int i=0; i<robot::num_links; i++)
    {
      link_translations[3*i+0] = T(robot::link_transform_translation_only[i][0]);
      link_translations[3*i+1] = T(robot::link_transform_translation_only[i][1]);
      link_translations[3*i+2] = T(robot::link_transform_translation_only[i][2]);
      link_rotations[4*i+0] = T(robot::link_transform_quaternion_only[i][0]);
      link_rotations[4*i+1] = T(robot::link_transform_quaternion_only[i][1]);
      link_rotations[4*i+2] = T(robot::link_transform_quaternion_only[i][2]);
      link_rotations[4*i+3] = T(robot::link_transform_quaternion_only[i][3]);
    }

    for (int i=0; i<robot::num_joints; i++)
    {
      int child_link_idx = robot::joint_child_link_idx[i];
      int parent_link_idx = robot::joint_parent_link_idx[i];
      int target_idx = joint_idx_to_target_idx[i];
      int variable_idx = robot::joint_idx_to_variable_idx[i];

      // init
      if (parent_link_idx == -1)
      {
        // TODO: convert to function
        global_link_translations[3*child_link_idx+0] = T(0.0);
        global_link_translations[3*child_link_idx+1] = T(0.0);
        global_link_translations[3*child_link_idx+2] = T(0.0);
        global_link_rotations[4*child_link_idx+0] = T(1.0);
        global_link_rotations[4*child_link_idx+1] = T(0.0);
        global_link_rotations[4*child_link_idx+2] = T(0.0);
        global_link_rotations[4*child_link_idx+3] = T(0.0);
        continue;
      }

      // Translation
      if (robot::link_can_skip_translation[child_link_idx])
      {
          // TODO: convert to function
          global_link_translations[3*child_link_idx+0] = global_link_translations[3*parent_link_idx+0];
          global_link_translations[3*child_link_idx+1] = global_link_translations[3*parent_link_idx+1];
          global_link_translations[3*child_link_idx+2] = global_link_translations[3*parent_link_idx+2];
      }
      else
      {
        utils::computeLinkTranslation(&(global_link_translations[3*parent_link_idx]), 
                                      &(global_link_rotations[4*parent_link_idx]), 
                                      &(link_translations[3*child_link_idx]), 
                                      &(global_link_translations[3*child_link_idx]));
      }

      if (variable_idx!=-1) // if joint can move
      { 
        T joint_val;
        if (target_idx!=-1)
        { // this is an optimization target
          joint_val = target_values[target_idx];
        }
        else
        { // this is a joint value but not an optimization target
          joint_val = T(variable_positions[variable_idx]);
        }

        if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then only rotate using the joint position
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]),
                                    joint_val, 
                                    &(global_link_rotations[4*child_link_idx]));
        }
        else // if link has rotation and joint has rotation, then we need to rotate using both
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                    &(link_rotations[4*child_link_idx]), 
                                    joint_val, 
                                    &(global_link_rotations[4*child_link_idx]));
        }
      }
      else // if joint is static
      {
        if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then no need to do anything
        {
          // TODO: convert to function
          global_link_rotations[4*child_link_idx+0] = global_link_rotations[4*parent_link_idx+0];
          global_link_rotations[4*child_link_idx+1] = global_link_rotations[4*parent_link_idx+1];
          global_link_rotations[4*child_link_idx+2] = global_link_rotations[4*parent_link_idx+2];
          global_link_rotations[4*child_link_idx+3] = global_link_rotations[4*parent_link_idx+3];
        }
        else // if link has a rotation, only compute that
        {
          utils::computeLinkRotation(&(global_link_rotations[4*parent_link_idx]), 
                                    &(link_rotations[4*child_link_idx]),
                                    &(global_link_rotations[4*child_link_idx]));
        }

      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    // TODO: testing collision
    const double dist_threshold = 0.1; // TODO: move to robot conf

    // Compute collision positions
    T collision_pos[3*countOf(robot::collisions)]; // w.r.t. world frame
    for (int i=0; i<countOf(robot::collisions); i++)
    {
    const robot::Collision &obj = robot::collisions[i];

    T obj_pos[3];  // w.r.t. related link frame
    obj_pos[0] = T(obj.x);
    obj_pos[1] = T(obj.y);
    obj_pos[2] = T(obj.z);
    utils::computeLinkTranslation(&(global_link_translations[3*obj.link_idx]),
                                  &(global_link_rotations[4*obj.link_idx]),
                                  obj_pos,
                                  &(collision_pos[3*i]));
    }

    // Compute collision pairs
    for (int i=0; i<robot::num_links; i++)
    {
      for (int j=0; j<robot::num_links; j++)
      {
        for (int k=0; k<countOf(robot::collisions); k++)
        {
          if (robot::processed_acm[i][j] == 0)
          {
            const robot::Collision &obj1 = robot::collisions[i];
            const robot::Collision &obj2 = robot::collisions[j];
            

            residuals[k] = utils::distSphere2Sphere(&(collision_pos[3*i]), obj1.radius, &(collision_pos[3*j]), obj2.radius);
            if (residuals[k]>dist_threshold) residuals[k] = T(dist_threshold);
            residuals[k] = dist_threshold - residuals[k];
          }
        }
      }
    }

    return true; // TODO: maybe return false if there is a collision?
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const int (&joint_idx_to_target_idx)[robot::num_joints], const double* variable_positions)
   {
     return (new ceres::AutoDiffCostFunction<CollisionAvoidanceGoal, COUNTOF(robot::collisions), robot::num_targets>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new CollisionAvoidanceGoal(joint_idx_to_target_idx, variable_positions)));
   }

  const int (&joint_idx_to_target_idx)[robot::num_joints];
  const double* variable_positions;
};


} // namespace salih_marangoz_thesis


#endif // __CERES_IK_H__