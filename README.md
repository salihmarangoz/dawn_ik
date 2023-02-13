# Salih Marangoz Thesis

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)



## Installation

- http://ceres-solver.org/installation.html (2.x.x)

- https://github.com/xArm-Developer/xarm_ros (master)

## Running

### Simulation

```bash
# select one!
$ roslaunch horti_moveit_config demo_gazebo.launch # TODO: modify the launch file
$ roslaunch salih_marangoz_thesis ceres_ik_sim.launch
```

### Code Generation

```bash
# 1. Make sure the robot description is loaded. (if the simulation is running then it is loaded)
# 2. This will generate the robot configuration header file. Manually copy the text out (not all of it)
$ rosrun salih_marangoz_thesis robot_parser_node
```

### IK Solver/Controller

```bash
$ roslaunch salih_marangoz_thesis ceres_ik_node.launch
```



## ToDo:

- **[EIGEN_NO_DEBUG](https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html)**: disables Eigen's assertions if defined. Not defined by default, unless the `NDEBUG` macro is defined (this is a standard C++ macro which disables all asserts).
- `-funroll-loops` or `-O3` is needed!
