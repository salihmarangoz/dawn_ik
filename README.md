# DawnIK

DawnIK [1]  is a real-time inverse kinematics solver for robotic arms focusing on observation capabilities with collision avoidance and multiple objectives. This repository contains the accompanying code for the paper "DawnIK: Decentralized Collision-Aware Inverse Kinematics Solver for Heterogeneous Multi-Arm Systems" by Salih Marangoz, Rohit Menon, Nils Dengler, Maren Bennewitz submitted for IEEE-RAS Humanoids 2023. you can find the paper at https://arxiv.org/abs/2307.12750

[![](https://img.youtube.com/vi/-k7XJkbAB6A/0.jpg)](https://www.youtube.com/watch?v=-k7XJkbAB6A)


## Goal Types

DawnIK has different goals defined in `goals.h` which can be combined specifically for a use-case. There are some examples:

- Position Goal: [[Video]](https://www.youtube.com/watch?v=zrl12iFnM6M)
- Position Goal + Orientation Goal: [[Video]](https://www.youtube.com/watch?v=_uTy60yxK6U)
- Look-at Goal (+ Position Goal with a small weight): [[Video]](https://www.youtube.com/watch?v=3-Y2mOZWGVc)

## Dependencies

```bash
$ cd catkin_ws/src

# This package
$ git clone git@github.com:salihmarangoz/dawn_ik.git 

# xArm ROS (optional)
$ git clone git@github.com:salihmarangoz/xarm_ros.git 
$ cd xarm_ros
$ git submodule update --init --remote

# Fake Joints (optional)
$ git clone git@github.com:salihmarangoz/fake_joint

# Collision evaluation (optional)
$ git clone git@github.com:salihmarangoz/moveit_collision_check.git

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

- Make sure that the robot joints don't have further position limits (e.g. for improving MoveIt's planninng behavior where it is limited between [-PI,+PI]).
- If you would like to use collisions objects other than spheres you need to modify the code. Modify `dawn_ik.cpp` around line 295 to create `CollisionAvoidanceGoalNumeric` instead of `CollisionAvoidanceGoal`. With this change, dawn_ik will use numerical diff instead of autodiff. Be careful because the convergence performance may be affected.
- DawnIK can be used to isolate an arm from the collision computations in an multi-arm system. This is usually done by disabling collisions for the controlled arm. However, for the ACM computations with MoveIt make sure all collisions are enabled.

Make sure the robot description is loaded. (if the fake/sim is running then it is probably loaded). Re-compile the project after this step. 

```bash
# 1. Option: Generate the header
$ rosrun dawn_ik robot_parser_node _cfg:=lite6 # for lite6.yaml

# 2. Option: Use a pre-generated header
$ roscd dawn_ik/include/dawn_ik/robot_configuration
$ cp lite6.h autogen_test.h

# Re-compile the project
$ catkin build
```

### Solver/Controller

Start the solver. This command will also launch RViz for providing input to the controller. There will be two interactive markers, one is for the endpoint pose and the other one is for the look-at goal. **Right click** one of the markers to set the current mode.

```bash
$ roslaunch dawn_ik lite6_solver.launch
```

## F.A.Q.

- Solver crashes:
  - Make sure to disable `horti_acm_tricks` if it is not Horti. `horti_acm_tricks` disabled collision checking between the external arms. So, if you have a multi-arm robotic system with more than 2 arms, you may need to adapt the code in `robot_parser.cpp` for your use case.

- Parser crashes:
  - Only single-axis revolute joints and static joints are supported.


## Footnotes

- [1] [Ceres Solver](http://ceres-solver.org/) is heavily used in this project so we named this project similar to [how Ceres Solver is named](http://ceres-solver.org/#f1). [Dawn](https://solarsystem.nasa.gov/missions/dawn/overview/) is the spacecraft launched in 2007 by NASA, reached to Ceres in 2015 and acquired the dwarf planet's information of global shape, mean density, surface morphology, mineralogy, etc. by the middle of 2016. 

