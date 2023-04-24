# Salih Marangoz Thesis

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)

## Latest Dev Preview

[![](https://img.youtube.com/vi/HZM6uRaNSIk/0.jpg)](https://www.youtube.com/watch?v=HZM6uRaNSIk)

## Installation

```bash
# Install Ceres Solver 2.x.x
# http://ceres-solver.org/installation.html

# xArm
$ git clone clone git@github.com:salihmarangoz/xarm_ros.git
$ cd xarm_ros
$ git submodule update --init --remote
```

## Running

### Simulation

```bash
# select one!
$ roslaunch salih_marangoz_thesis lite6_sim.launch
$ roslaunch salih_marangoz_thesis xarm5_sim.launch
$ roslaunch salih_marangoz_thesis xarm6_sim.launch
$ roslaunch salih_marangoz_thesis xarm7_sim.launch
$ roslaunch salih_marangoz_thesis horti_sim.launch
```

### Code Generation

Make sure the robot description is loaded. (if the simulation is running then it is loaded). Re-compile the project after this step. 

```bash
# select one!
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=lite6
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=xarm5
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=xarm6
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=xarm7
$ rosrun salih_marangoz_thesis robot_parser_node _cfg:=horti

# Re-compile the project after this step. 
```

### Solver/Controller

```bash
# select one!
$ roslaunch salih_marangoz_thesis lite6_solver.launch
$ roslaunch salih_marangoz_thesis xarm5_solver.launch
$ roslaunch salih_marangoz_thesis xarm6_solver.launch
$ roslaunch salih_marangoz_thesis xarm7_solver.launch
$ roslaunch salih_marangoz_thesis horti_solver.launch
```
