cmake_minimum_required(VERSION 3.0.2)
project(dawn_ik)

add_compile_options(-std=c++17)

## Further optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wno-int-to-pointer-cast")

## Disable assertions for more performance (maybe a little)
#set(EIGEN_NO_DEBUG "1")
#set(NDEBUG "1")

# Memory error sanitizer
#add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
#add_link_options(-fsanitize=address -fno-omit-frame-pointer)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2
  tf2_eigen
  visualization_msgs
  message_generation
  sensor_msgs
  control_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Ceres REQUIRED COMPONENTS EigenSparse SparseLinearAlgebraLibrary LAPACK SuiteSparse SchurSpecializations)
find_package(hpp-fcl REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
  LinkState.msg
  CollisionState.msg
  JointLinkState.msg
  JointLinkCollisionState.msg
  SolverSummary.msg
  IKGoal.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${hpp-fcl_INCLUDE_DIRS}
)

add_executable(solver_node
  src/nodes/solver_node.cpp
  src/dawn_ik.cpp
  src/joint_trajectory_control_interface.cpp
  src/robot_monitor.cpp
)
target_compile_definitions(solver_node PRIVATE -DUSE_MICK_OTHER=0)
add_dependencies(solver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(solver_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${hpp-fcl_LIBRARIES}
  Ceres::ceres
)

add_executable(robot_parser_node 
  src/nodes/robot_parser_node.cpp
  src/robot_parser.cpp
  src/yaml.cpp
)
add_dependencies(robot_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_parser_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
