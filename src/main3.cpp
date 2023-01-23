
#include <stdio.h>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"


#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;

struct ForwardKinematicsError {
  ForwardKinematicsError(const std::vector<Eigen::Isometry3d> &transforms, const Eigen::Vector3d &endpoint)
      : transforms(transforms) , endpoint(endpoint) {}

  template <typename T>
  bool operator()(const T* joint_values, T* residuals) const // param_x, param_y, residuals
  {

    Eigen::Matrix<T,3,1> final_translation; final_translation << T(0), T(0), T(0); // todo: init
    Eigen::Quaternion<T> final_rotation; final_rotation.setIdentity(); // todo: init

    // ---------------------------------------------------
    auto R0 = transforms[0].rotation();
    auto T0_ = transforms[0].translation();
    Eigen::Matrix<T,3,1> T0; T0 << T(T0_[0]), T(T0_[1]), T(T0_[2]);
    auto J0 = Eigen::AngleAxis<T>(joint_values[0], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T0;
    final_rotation =  final_rotation * R0 * J0;

    // ---------------------------------------------------
    auto R1 = transforms[1].rotation();
    auto T1_ = transforms[1].translation();
    Eigen::Matrix<T,3,1> T1; T1 << T(T1_[0]), T(T1_[1]), T(T1_[2]);
    auto J1 = Eigen::AngleAxis<T>(joint_values[1], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T1;
    final_rotation =  final_rotation * R1 * J1;

    // ---------------------------------------------------
    auto R2 = transforms[2].rotation();
    auto T2_ = transforms[2].translation();
    Eigen::Matrix<T,3,1> T2; T2 << T(T2_[0]), T(T2_[1]), T(T2_[2]);
    auto J2 = Eigen::AngleAxis<T>(joint_values[2], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T2;
    final_rotation =  final_rotation * R2 * J2;

    // ---------------------------------------------------
    auto R3 = transforms[3].rotation();
    auto T3_ = transforms[3].translation();
    Eigen::Matrix<T,3,1> T3; T3 << T(T3_[0]), T(T3_[1]), T(T3_[2]);
    auto J3 = Eigen::AngleAxis<T>(joint_values[3], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T3;
    final_rotation =  final_rotation * R3 * J3;

    // ---------------------------------------------------
    auto R4 = transforms[4].rotation();
    auto T4_ = transforms[4].translation();
    Eigen::Matrix<T,3,1> T4; T4 << T(T4_[0]), T(T4_[1]), T(T4_[2]);
    auto J4 = Eigen::AngleAxis<T>(joint_values[4], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T4;
    final_rotation =  final_rotation * R4 * J4;

    // ---------------------------------------------------
    auto R5 = transforms[5].rotation();
    auto T5_ = transforms[5].translation();
    Eigen::Matrix<T,3,1> T5; T5 << T(T5_[0]), T(T5_[1]), T(T5_[2]);
    auto J5 = Eigen::AngleAxis<T>(joint_values[5], Eigen::Matrix<T,3,1>::UnitZ()).toRotationMatrix();

    final_translation = final_translation + final_rotation * T5;
    final_rotation =  final_rotation * R5 * J5;

    residuals[0] = final_translation[0] - endpoint[0];
    residuals[1] = final_translation[1] - endpoint[1];
    residuals[2] = final_translation[2] - endpoint[2];
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const std::vector<Eigen::Isometry3d> &transforms, const Eigen::Vector3d &endpoint)
   {
     return (new ceres::AutoDiffCostFunction<ForwardKinematicsError, 3, 6>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new ForwardKinematicsError(transforms, endpoint)));
   }

  const std::vector<Eigen::Isometry3d> transforms;
  const Eigen::Vector3d endpoint;
};




class InverseKinematics
{
public:
  InverseKinematics(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh)
  {
    endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 1, &InverseKinematics::subscriberCallback, this);
  
    // Load the planning scene monitor
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
      exit(-1);
    }

    // Start the planning scene monitor
    planning_scene_monitor->startSceneMonitor();
    // planning_scene_monitor->startWorldGeometryMonitor(
    //     planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
    //     planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
    //     false /* skip octomap monitor */);
    //planning_scene_monitor->setStateUpdateFrequency(100);
    planning_scene_monitor->startStateMonitor();

    model = planning_scene_monitor->getRobotModel();

    visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(model->getModelFrame(),"/moveit_visual_markers"));
    //visual_tools->setPlanningSceneMonitor(planning_scene_monitor);

  }

  const std::vector<Eigen::Isometry3d>& getTransforms()
  {
    // Cache transforms
    if (cached_transforms.size() == 0)
    {
      auto prev_joint = model->getJointModel(model->getRootJoint()->getName());
      prev_joint = prev_joint->getChildLinkModel()->getChildJointModels()[0];
      while (true)
      {
        auto current_link = prev_joint->getChildLinkModel();
        if (current_link->getChildJointModels().size() <= 0) break;
        auto current_joint = current_link->getChildJointModels()[0];

        auto t = current_link->getJointOriginTransform();
        cached_transforms.push_back(t);

        prev_joint = current_joint;
      }
    }

    return cached_transforms;
  }

  std::vector<double> getJointValues(moveit::core::RobotState robot_state)
  {
    std::vector<double> joint_values;

    auto prev_joint = model->getJointModel(model->getRootJoint()->getName());
    prev_joint = prev_joint->getChildLinkModel()->getChildJointModels()[0];

    while (true)
    {
      auto current_link = prev_joint->getChildLinkModel();
      if (current_link->getChildJointModels().size() <= 0) break;
      auto current_joint = current_link->getChildJointModels()[0];

      double joint_angle = robot_state.getJointPositions(prev_joint->getName())[0];
      joint_values.push_back(joint_angle);

      prev_joint = current_joint;
    }

    return joint_values;
  }

  void setJointValues(moveit::core::RobotState &robot_state, const std::vector<double> joint_values)
  {
    auto prev_joint = model->getJointModel(model->getRootJoint()->getName());
    prev_joint = prev_joint->getChildLinkModel()->getChildJointModels()[0];
    
    int cur_idx = 0;
    while (true)
    {
      auto current_link = prev_joint->getChildLinkModel();
      if (current_link->getChildJointModels().size() <= 0) break;
      auto current_joint = current_link->getChildJointModels()[0];

      double joint_angle = robot_state.getJointPositions(prev_joint->getName())[0];
      robot_state.setJointPositions(prev_joint->getName(), &(joint_values[cur_idx]));

      prev_joint = current_joint;
      cur_idx++;
    }
  }

  void subscriberCallback(const visualization_msgs::InteractiveMarkerUpdatePtr &msg)
  {
    if (msg->poses.size()>0)
    {
      given_endpoint = msg->poses[0].pose;
      ROS_WARN_THROTTLE(1.0, "Endpoint received. x: %lf, y: %lf, z: %lf", given_endpoint.position.x, given_endpoint.position.y, given_endpoint.position.z);

      // SLOW
      //planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
      //moveit::core::RobotState current_state = lps->getCurrentState(); // copy the current state

      // FAST BUT DANGEROUS
      planning_scene_monitor->waitForCurrentRobotState(ros::Time(0));

      // TODO: STATIC FOR TESTING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      static moveit::core::RobotState current_state = *(planning_scene_monitor->getStateMonitor()->getCurrentState()); // copy the current state

      auto transforms = getTransforms();
      Eigen::Vector3d endpoint;
      endpoint.x() =  given_endpoint.position.x;
      endpoint.y() =  given_endpoint.position.y;
      endpoint.z() =  given_endpoint.position.z;
      std::vector<double> joint_values = getJointValues(current_state);

      ceres::Problem problem;
      ceres::CostFunction* cost_function = ForwardKinematicsError::Create(transforms, endpoint);
      problem.AddResidualBlock(cost_function, nullptr /* squared loss */, joint_values.data());

      // problem.SetParameterLowerBound(parameters, 0, 0.02); problem.SetParameterUpperBound(parameters, 0, 0.15); // TODO
      
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_SCHUR;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.FullReport() << "\n";

      //auto joint_values = getJointValues(current_state); // For testing
      setJointValues(current_state, joint_values);

      visual_tools->publishRobotState(current_state);

    }
  }

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  ros::Subscriber endpoint_sub;
  geometry_msgs::Pose given_endpoint;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  collision_detection::AllowedCollisionMatrix acm;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  moveit::core::RobotModelConstPtr model;
  std::vector<Eigen::Isometry3d> cached_transforms;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "salih_marangoz_inverse_kinematics_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  InverseKinematics ik(nh, priv_nh);
  //ros::spin();
  ros::waitForShutdown();
  return 0;
}



/*
int main(int argc, char** argv) {
  IKSolver solver;
  solver.setDragPoint(Point3D(0.0, 0.5, 0.5));
  solver.setScreenPoint(Point2D(0.5, 0.5));
  solver.stepSolve(10);
  double angle = solver.getJointValue(0);
  cout << "angle = " << angle << "\n";
  double mat[9];
  double angle_axis[3] = {0, 0, angle};
  ceres::AngleAxisToRotationMatrix(angle_axis, mat);
  return 0;
}
*/