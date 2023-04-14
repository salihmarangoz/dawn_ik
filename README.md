# Salih Marangoz Thesis

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)



## Installation

- http://ceres-solver.org/installation.html (2.1.0)

- https://github.com/xArm-Developer/xarm_ros (master) (commit: c10d5e59dd50109d15e8fbd3da0bae6aeacbb855)

## Running

### Simulation

Select one!

#### 1. xArm7

Note: To increase joint states update frequency, added `<param name="rate" value="200" />` to the `joint_state_publisher` in `xarm_ros/xarm7_moveit_config/launch/moveit_rviz_common.launch`.

```bash
$ roslaunch salih_marangoz_thesis xarm7_sim.launch
```

#### 2. Horti

```bash
$ roslaunch horti_moveit_config demo_gazebo.launch # TODO: modify the launch file
```

### Code Generation

Make sure the robot description is loaded. (if the simulation is running then it is loaded). Re-compile the project after this step. 

```bash
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=xarm7
# or
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=horti
```

### Solver/Controller

```bash
$ roslaunch salih_marangoz_thesis xarm7_solver.launch
# or
$ roslaunch salih_marangoz_thesis horti_solver.launch
```
