
#include <stdio.h>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
#include "matrices.h"
#include "transforms.h"

#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
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
    auto J0 = Eigen::AngleAxis<T>(joint_values[0], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T0;
    final_rotation =  final_rotation * R0 * J0;

    // ---------------------------------------------------
    auto R1 = transforms[1].rotation();
    auto T1_ = transforms[1].translation();
    Eigen::Matrix<T,3,1> T1; T1 << T(T1_[0]), T(T1_[1]), T(T1_[2]);
    auto J1 = Eigen::AngleAxis<T>(joint_values[1], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T1;
    final_rotation =  final_rotation * R1 * J1;

    // ---------------------------------------------------
    auto R2 = transforms[2].rotation();
    auto T2_ = transforms[2].translation();
    Eigen::Matrix<T,3,1> T2; T2 << T(T2_[0]), T(T2_[1]), T(T2_[2]);
    auto J2 = Eigen::AngleAxis<T>(joint_values[2], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T2;
    final_rotation =  final_rotation * R2 * J2;

    // ---------------------------------------------------
    auto R3 = transforms[3].rotation();
    auto T3_ = transforms[3].translation();
    Eigen::Matrix<T,3,1> T3; T3 << T(T3_[0]), T(T3_[1]), T(T3_[2]);
    auto J3 = Eigen::AngleAxis<T>(joint_values[3], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T3;
    final_rotation =  final_rotation * R3 * J3;

    // ---------------------------------------------------
    auto R4 = transforms[4].rotation();
    auto T4_ = transforms[4].translation();
    Eigen::Matrix<T,3,1> T4; T4 << T(T4_[0]), T(T4_[1]), T(T4_[2]);
    auto J4 = Eigen::AngleAxis<T>(joint_values[4], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T4;
    final_rotation =  final_rotation * R4 * J4;

    // ---------------------------------------------------
    auto R5 = transforms[5].rotation();
    auto T5_ = transforms[5].translation();
    Eigen::Matrix<T,3,1> T5; T5 << T(T5_[0]), T(T5_[1]), T(T5_[2]);
    auto J5 = Eigen::AngleAxis<T>(joint_values[5], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T5;
    final_rotation =  final_rotation * R5 * J5;

    // ---------------------------------------------------
    auto R6 = transforms[6].rotation();
    auto T6_ = transforms[6].translation();
    Eigen::Matrix<T,3,1> T6; T6 << T(T6_[0]), T(T6_[1]), T(T6_[2]);
    auto J6 = Eigen::AngleAxis<T>(joint_values[6], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T6;
    final_rotation =  final_rotation * R6 * J6;

    // ---------------------------------------------------
    auto R7 = transforms[7].rotation();
    auto T7_ = transforms[7].translation();
    Eigen::Matrix<T,3,1> T7; T7 << T(T7_[0]), T(T7_[1]), T(T7_[2]);
    auto J7 = Eigen::AngleAxis<T>(joint_values[7], Eigen::Matrix<T,3,1>::UnitZ());

    final_translation = final_translation + final_rotation * T7;
    final_rotation =  final_rotation * R7 * J7;

    residuals[0] = final_translation[0] - endpoint[0];
    residuals[1] = final_translation[1] - endpoint[1];
    residuals[2] = final_translation[2] - endpoint[2];
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const std::vector<Eigen::Isometry3d> &transforms, const Eigen::Vector3d &endpoint)
   {
     return (new ceres::AutoDiffCostFunction<ForwardKinematicsError, 3, 8>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new ForwardKinematicsError(transforms, endpoint)));
   }

  const std::vector<Eigen::Isometry3d> transforms;
  const Eigen::Vector3d endpoint;
};

/*
struct JointCenter {
  JointCenter(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
  : lower_bounds(lower_bounds), upper_bounds(upper_bounds) {}

  template <typename T>
  bool operator()(const T* joint_values, T* residuals) const // param_x, param_y, residuals
  {
    residuals[0] = joint_values[0] - (upper_bounds[0] - lower_bounds[0]) * 0.5;
    residuals[1] = joint_values[1] - (upper_bounds[1] - lower_bounds[1]) * 0.5;
    residuals[2] = joint_values[2] - (upper_bounds[2] - lower_bounds[2]) * 0.5;
    residuals[3] = joint_values[3] - (upper_bounds[3] - lower_bounds[3]) * 0.5;
    residuals[4] = joint_values[4] - (upper_bounds[4] - lower_bounds[4]) * 0.5;
    residuals[5] = joint_values[5] - (upper_bounds[5] - lower_bounds[5]) * 0.5;
    residuals[6] = joint_values[6] - (upper_bounds[6] - lower_bounds[6]) * 0.5;
    residuals[7] = joint_values[7] - (upper_bounds[7] - lower_bounds[7]) * 0.5;

    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const std::vector<double> &lower_bounds, const std::vector<double> &upper_bounds)
   {
     return (new ceres::AutoDiffCostFunction<JointCenter, 8, 8>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new JointCenter(lower_bounds, upper_bounds)));
   }
   
   std::vector<double> lower_bounds;
   std::vector<double> upper_bounds;
};
*/

/*
struct JointMovement {
  JointMovement(const std::vector<double> &init_values)
  :init_values(init_values) {}

  template <typename T>
  bool operator()(const T* joint_values, T* residuals) const // param_x, param_y, residuals
  {
    residuals[0] = joint_values[0] - init_values[0];
    residuals[1] = joint_values[1] - init_values[1];
    residuals[2] = joint_values[2] - init_values[2];
    residuals[3] = joint_values[3] - init_values[3];
    residuals[4] = joint_values[4] - init_values[4];
    residuals[5] = joint_values[5] - init_values[5];
    residuals[6] = joint_values[6] - init_values[6];
    residuals[7] = joint_values[7] - init_values[7];

    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const std::vector<double> &init_values)
   {
     return (new ceres::AutoDiffCostFunction<JointMovement, 8, 8>(  // num_of_residuals, size_param_x, size_param_y, ...
                 new JointMovement(init_values)));
   }
   
   std::vector<double> init_values;
};
*/


class IKSolver {
  
public:
  
  double width_;
  double height_;
  double robot_state_[7];
  double screen_x_;
  double screen_y_;
  double drag_x_;
  double drag_y_;
  double drag_z_;
  // NOTE: camera matrix is composed of camera extrinsic and intrinsic matrices
  Mat4 cameraMatrix_;
  vector<boost::shared_ptr<Transform> > transforms;
  Problem problem_; 
  int num_joints_;
  int start_transform_index_;
  
  IKSolver() : start_transform_index_(0), num_joints_(0), screen_x_(0.0), screen_y_(0.0), drag_x_(0.0), drag_y_(0.0), drag_z_(0.0) {
    buildProblem();
  }
  
  // defined later in this file
  // ugly, but lets me avoid separate class declaration and definition, which would slow me down
  void buildProblem();

  int getNumTransforms() { return transforms.size(); }

  double getJointValue(int idx) { return robot_state_[idx]; } 

  void addJointTransform(double lower_bound, double upper_bound) {
      cout << "Adding joint transform number " << num_joints_ << endl;
      problem_.SetParameterLowerBound(robot_state_, num_joints_, lower_bound);
      problem_.SetParameterUpperBound(robot_state_, num_joints_, upper_bound);
      transforms.push_back(boost::make_shared<JointTransform>(num_joints_++));
  }
  void addStaticTransform(Mat4 m)  {
      transforms.push_back(boost::make_shared<StaticTransform>(m));
  }
  void setCameraMatrix(Mat4 m)  { cameraMatrix_ = m; }

  void setStartTransformIndex(int idx) { start_transform_index_ = idx; }
  
  Point2D projectDragPoint() {
    Eigen::Matrix<double,4,1> drag_point(drag_x_, drag_y_, drag_z_, 1.0);
    Eigen::Matrix<double,2,1> pt(0.0, 0.0);
    // apply projection
    projectPoint(drag_point, pt);
    // copy to output
    Point2D point;
    point.x = pt(0);
    point.y = pt(1);
    return point;
  }

  template<typename T> inline void projectPoint(const Eigen::Matrix<T,4,1> & pt, Eigen::Matrix<T,2,1> & out) {
    Eigen::Matrix<T,4,4> P;
    matToEigen(cameraMatrix_, P);
    // apply projection
    Eigen::Matrix<T,4,1> x = P * pt;
    // from normalized device coords to screen pixel coords
    out(0) = (  x(0) / x(3) + T(1) ) * T(width_  / 2.0);
    out(1) = ( -x(1) / x(3) + T(1) ) * T(height_ / 2.0);
  }

  void setDims(Point2D dims) {
    width_ = dims.x;
    height_ = dims.y;
  }

  Point2D getScreenPoint() const { return Point2D(screen_x_, screen_y_); }
  void setScreenPoint(Point2D screen_point) { screen_x_ = screen_point.x;
                                              screen_y_ = screen_point.y; }
  
  Point3D getDragPoint() const { return Point3D(drag_x_, drag_y_, drag_z_); }
  void setDragPoint(Point3D drag_point) { drag_x_ = drag_point.x;
                                          drag_y_ = drag_point.y;
                                          drag_z_ = drag_point.z; }

  void stepSolve(int step_limit)
  {
    // Run the solver!
    Solver::Options options;
    options.max_num_iterations = step_limit;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
    cout << summary.FullReport() << "\n";
  }
  
  void timeSolve(double time_limit)
  {
    // Run the solver!
    Solver::Options options;
    options.max_solver_time_in_seconds = time_limit;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_type = ceres::LINE_SEARCH;
    Solver::Summary summary;
    Solve(options, &problem_, &summary);
  }
};


struct CostFunctor
{
  IKSolver* solver_;

  CostFunctor(IKSolver* solver)
      : solver_(solver) {}

  template <typename T> bool operator()(const T* const robot_state, T* residual) const
  {
    // Objective function: residual = Rx - s

    Eigen::Matrix<T,4,1> v(T(solver_->drag_x_), T(solver_->drag_y_), T(solver_->drag_z_), T(1.0));
    Eigen::Matrix<T,4,1> v_tmp;

    //T* state;
    //T tmp = T(*angle);
    //state = &tmp;

    for (int i = solver_->start_transform_index_; i >= 0; --i)
    {
        //solver_->transforms[i]->apply( state, v, v_tmp );
        solver_->transforms[i]->apply( robot_state, v, v_tmp );
        v = v_tmp;
    }

    // Camera projection
    Eigen::Matrix<T,2,1> pt_cam(T(0.0), T(0.0));
    solver_->projectPoint(v, pt_cam);

    residual[0] = pt_cam(0) - T(solver_->screen_x_);
    residual[1] = pt_cam(1) - T(solver_->screen_y_);

    return true;
  }

};

void IKSolver::buildProblem()
{
    CostFunction* cost_function =
        new AutoDiffCostFunction<CostFunctor, 2, 7>(new CostFunctor(this));
    problem_.AddResidualBlock(cost_function, NULL, robot_state_);
}

class InverseKinematics
{
public:
  InverseKinematics(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh)
  {
    endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1, &InverseKinematics::subscriberCallback, this);
  
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
        if (current_link->getChildJointModels().size() <= 0)
        {
                ROS_WARN("link: %s", current_link->getName().c_str());
          cached_transforms.push_back(current_link->getJointOriginTransform());
          break;
        }
        auto current_joint = current_link->getChildJointModels()[0];

        ROS_WARN("link: %s", current_link->getName().c_str());
        ROS_WARN("joint: %s", prev_joint->getName().c_str());

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

  const std::vector<double>& getJointUpperLimits()
  {
    if (cached_joint_upper_limits.size() == 0)
    {
      auto prev_joint = model->getJointModel(model->getRootJoint()->getName());
      prev_joint = prev_joint->getChildLinkModel()->getChildJointModels()[0];

      while (true)
      {
        auto current_link = prev_joint->getChildLinkModel();
        if (current_link->getChildJointModels().size() <= 0) break;
        auto current_joint = current_link->getChildJointModels()[0];

        auto bounds = prev_joint->getVariableBounds();
        if (bounds.size() == 0)
        {
          cached_joint_upper_limits.push_back(99999.0); // TODO
        }
        else
        {
          cached_joint_upper_limits.push_back(bounds[0].max_position_);
        }
        
        prev_joint = current_joint;
      }
    }
    return cached_joint_upper_limits;
  }

  const std::vector<double>& getJointLowerLimits()
  {
    if (cached_joint_lower_limits.size() == 0)
    {
      auto prev_joint = model->getJointModel(model->getRootJoint()->getName());
      prev_joint = prev_joint->getChildLinkModel()->getChildJointModels()[0];

      while (true)
      {
        auto current_link = prev_joint->getChildLinkModel();
        if (current_link->getChildJointModels().size() <= 0) break;
        auto current_joint = current_link->getChildJointModels()[0];

        auto bounds = prev_joint->getVariableBounds();
        if (bounds.size() == 0)
        {
          cached_joint_lower_limits.push_back(-99999.0); // TODO
        }
        else
        {
          cached_joint_lower_limits.push_back(bounds[0].min_position_);
        }

        prev_joint = current_joint;
      }
    }
    return cached_joint_lower_limits;
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

  void subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg)
  {
    given_endpoint = msg->pose;
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

    auto lower_bounds = getJointLowerLimits();
    auto upper_bounds = getJointUpperLimits();
    for (int i=0; i<joint_values.size(); i++)
    {
      //ROS_WARN("idx: %d", i);
      problem.SetParameterLowerBound(joint_values.data(), i, lower_bounds[i]); 
      problem.SetParameterUpperBound(joint_values.data(), i, upper_bounds[i]);
    }

    ceres::CostFunction* cost_function2 = JointCenter::Create(lower_bounds, upper_bounds);
    ceres::CauchyLoss *h = new ceres::CauchyLoss(1.0);
    problem.AddResidualBlock(cost_function2, h, joint_values.data());

    std::vector<double> joint_values_copy = joint_values;
    ceres::CostFunction* cost_function3 = JointMovement::Create(joint_values_copy);
    ceres::CauchyLoss *h2 = new ceres::CauchyLoss(1.0);
    problem.AddResidualBlock(cost_function3, h2, joint_values.data());
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::TRUST_REGION; // LINE_SEARCH methods don't support bounds
    options.minimizer_progress_to_stdout = false;

    
    // experimental
    //options.preconditioner_type = ceres::SUBSET;
    //options.jacobi_scaling = true; // TODO
    //options.use_nonmonotonic_steps = true;
    //options.use_approximate_eigenvalue_bfgs_scaling = true;
    //options.use_mixed_precision_solves = true; options.max_num_refinement_iterations = 3;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    //auto joint_values = getJointValues(current_state); // For testing
    setJointValues(current_state, joint_values);

    visual_tools->publishRobotState(current_state);

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
  std::vector<double> cached_joint_upper_limits;
  std::vector<double> cached_joint_lower_limits;
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