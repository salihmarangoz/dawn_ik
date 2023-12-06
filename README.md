# DawnIK

DawnIK [1]  is a real-time inverse kinematics solver for robotic arms focusing on observation capabilities with collision avoidance and multiple objectives. This repository contains the accompanying code for the paper "DawnIK: Decentralized Collision-Aware Inverse Kinematics Solver for Heterogeneous Multi-Arm Systems" by Salih Marangoz, Rohit Menon, Nils Dengler, Maren Bennewitz submitted for IEEE-RAS Humanoids 2023. you can find the paper at https://arxiv.org/abs/2307.12750

## Video

[![](https://img.youtube.com/vi/-k7XJkbAB6A/0.jpg)](https://www.youtube.com/watch?v=-k7XJkbAB6A)


## Dependencies

```bash
$ cd catkin_ws/src

# This package
$ git clone git@github.com:salihmarangoz/dawn_ik.git 

# xArm ROS (optional)
$ git clone clone git@github.com:salihmarangoz/xarm_ros.git 
$ cd xarm_ros
$ git submodule update --init --remote

# Fake Joints (optional)
$ git clone https://github.com/salihmarangoz/fake_joint

# Collision evaluation (optional)
$ git@github.com:salihmarangoz/moveit_collision_check.git

# Ceres Solver 2.x.x (http://ceres-solver.org/installation.html)
$ cd $HOME
$ git clone git@github.com:ceres-solver/ceres-solver.git -b 2.2.0rc1
$ sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
$ cd ceres-solver
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF ..
$ make -j8
$ sudo make install
$ sudo ldconfig # Update dynamic linker run-time bindings

# Other packages
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src -r
$ sudo apt install python3-yaml python-is-python3
$ pip install pyyaml
```

## Running

### Simulation

```bash
# Start with gazebo
$ roslaunch dawn_ik lite6_sim.launch
# or using fake_joints
$ roslaunch dawn_ik lite6_fake.launch
```

### Code Generation

Make sure the robot description is loaded. (if gazebo/fake_joints is running then it is probably loaded). Re-compile the project after this step. 

```bash
# Generate the header
$ rosrun dawn_ik robot_parser_node _cfg:=lite6

# Use a pre-generated header
$ roscd dawn_ik/include/dawn_ik/robot_configuration
$ cp lite6.h autogen_test.h

# Re-compile the project
$ catkin build
```

### Solver/Controller

```bash
$ roslaunch dawn_ik lite6_solver.launch
```


### Footnotes

- [1] [Dawn](https://solarsystem.nasa.gov/missions/dawn/overview/) is the spacecraft launched in 2007 by NASA, reached to Ceres in 2015 and acquired the dwarf planet's information of global shape, mean density, surface morphology, mineralogy, etc. by the middle of 2016. 

