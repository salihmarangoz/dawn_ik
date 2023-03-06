# Salih Marangoz Thesis

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)



## Installation

- http://ceres-solver.org/installation.html (2.x.x)

- https://github.com/xArm-Developer/xarm_ros (master)

## Running

### Simulation

Note: To increase joint states update frequency, added `<param name="rate" value="200" />` to the `joint_state_publisher` in `xarm_ros/xarm7_moveit_config/launch/moveit_rviz_common.launch`.

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
