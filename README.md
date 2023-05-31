# DawnIK Solver (Salih Marangoz Thesis)

DawnIK Solver [1]  is a real-time inverse kinematics solver for robotic arms focusing on observation capabilities with collision avoidance and multiple objectives.

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)

## Latest Dev Preview

[![](https://img.youtube.com/vi/HZM6uRaNSIk/0.jpg)](https://www.youtube.com/watch?v=HZM6uRaNSIk)

## Dependencies

```bash
################# ROS DEPENDENCIES ##################################

$ cd catkin_ws/src

# This package
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/salih_marangoz_thesis.git

# Horti Robot
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/horti_model.git -b salih_master_thesis

# xArm ROS
$ git clone clone git@github.com:salihmarangoz/xarm_ros.git
$ cd xarm_ros
$ git submodule update --init --remote

# Others
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src -r

################## EXTERNAL DEPENDENCIES ############################

# Ceres Solver 2.x.x (http://ceres-solver.org/installation.html)
$ cd $HOME
$ git clone git@github.com:salihmarangoz/ceres-solver.git
$ sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
$ cd ceres-solver
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make -j8
$ sudo make install

# Ruckig
$ cd $HOME
$ git clone git@github.com:salihmarangoz/ruckig.git
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make -j8
$ sudo make install

# Extra stuff
$ sudo apt install python3-yaml python-is-python3
$ pip install pyyaml

# Update dynamic linker run-time bindings
sudo ldconfig
```

## Running

### Simulation

```bash
# select one!
$ roslaunch dawn_ik lite6_sim.launch
$ roslaunch dawn_ik xarm5_sim.launch
$ roslaunch dawn_ik xarm6_sim.launch
$ roslaunch dawn_ik xarm7_sim.launch
$ roslaunch dawn_ik horti_sim.launch
```

### Code Generation

Make sure the robot description is loaded. (if the simulation is running then it is loaded). Re-compile the project after this step. 

```bash
# select one!
$ rosrun dawn_ik robot_parser_node _cfg:=lite6
$ rosrun dawn_ik robot_parser_node _cfg:=xarm5
$ rosrun dawn_ik robot_parser_node _cfg:=xarm6
$ rosrun dawn_ik robot_parser_node _cfg:=xarm7
$ rosrun dawn_ik robot_parser_node _cfg:=horti

# Re-compile the project after this step. 
```

### Solver/Controller

```bash
# select one!
$ roslaunch dawn_ik lite6_solver.launch
$ roslaunch dawn_ik xarm5_solver.launch
$ roslaunch dawn_ik xarm6_solver.launch
$ roslaunch dawn_ik xarm7_solver.launch
$ roslaunch dawn_ik horti_solver.launch
```

### Footnotes

- [1] [Ceres Solver](http://ceres-solver.org/) is heavily used in this project so we named this project similar to [how Ceres Solver is named](http://ceres-solver.org/#f1). [Dawn](https://solarsystem.nasa.gov/missions/dawn/overview/) is the spacecraft launched in 2007 by NASA, reached to Ceres in 2015 and acquired the dwarf planet's information of global shape, mean density, surface morphology, mineralogy, etc. by the middle of 2016. 

