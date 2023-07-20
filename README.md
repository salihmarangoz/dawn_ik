# DawnIK Solver (Salih Marangoz Thesis)

DawnIK Solver [1]  is a real-time inverse kinematics solver for robotic arms focusing on observation capabilities with collision avoidance and multiple objectives.

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)

## Latest Dev Preview

[![](https://img.youtube.com/vi/2WPIzhGtnZw/0.jpg)](https://www.youtube.com/watch?v=2WPIzhGtnZw)

Previous recordings:
- https://www.youtube.com/watch?v=HZM6uRaNSIk

## Dependencies

```bash
################# ROS DEPENDENCIES ##################################
$ cd catkin_ws/src

# This package
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/salih_marangoz_thesis.git

# Horti Robot (use salih_master_thesis branch!)
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/horti_model.git -b salih_master_thesis

# Mick Robot (modified copy of horti_model package)
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/mick_model.git

# xArm ROS (forked)
$ git clone clone git@github.com:salihmarangoz/xarm_ros.git
$ cd xarm_ros
$ git submodule update --init --remote

# Fake Joints (optional alternative to Gazebo) (forked and modified)
$ git clone https://github.com/salihmarangoz/fake_joint

# OPTIONAL: Collision IK
# Includes trained model for lite6 (forked and heavily modified)
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/collision_ik.git

# OPTIONAL: For robot state collision evaluation
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/moveit_collision_check.git

# Others
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src -r
$ sudo apt install python3-yaml python-is-python3
$ pip install pyyaml

################## EXTERNAL DEPENDENCIES ############################

# Ceres Solver 2.x.x (http://ceres-solver.org/installation.html)
$ cd $HOME
$ git clone git@github.com:salihmarangoz/ceres-solver.git
$ sudo apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
$ cd ceres-solver
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF ..
$ make -j8
$ sudo make install

# Update dynamic linker run-time bindings
sudo ldconfig
```

## Running

### Simulation

```bash
# select one!
$ roslaunch dawn_ik lite6_sim.launch
$ roslaunch dawn_ik horti_sim.launch
#$ roslaunch dawn_ik mick_sim.launch TODO

# or without Gazebo (using fake_joints)
$ roslaunch dawn_ik lite6_fake.launch
$ roslaunch dawn_ik horti_fake.launch
$ roslaunch dawn_ik mick_fake.launch
```

### Code Generation

**BE CAREFUL:** MAKE SURE JOINTS DONT HAVE EXTRA POSITION LIMITS. SOME CONFIGURATIONS LIMIT JOINT POSITIONS BETWEEN [-PI,+PI] FOR MORE STABLE MOVEIT SOLUTIONS. (See horti_model repository's salih_marangoz_thesis branch as an example and check the README.md)

**ALSO:** For experiments, we disable head arm's collision in general. But this intervenes with the ACM. We recommend enabling all collisions (see horti_macro.xacro -> `experiment` property)

Make sure the robot description is loaded. (if the fake/sim is running then it is probably loaded). Re-compile the project after this step. 

```bash
# To skip the code generation step, replace autogen_test.h with pre-generated headers (lite6.h, horti.h, etc.)

# select one!
$ rosrun dawn_ik robot_parser_node _cfg:=lite6
$ rosrun dawn_ik robot_parser_node _cfg:=horti
$ rosrun dawn_ik robot_parser_node _cfg:=mick

# Re-compile the project after this step. 
```

### Solver/Controller

DawnIK and CollisionIK (with goal+collision adapter) are available.

```bash
# If you would like to use our solver, select one!
$ roslaunch dawn_ik lite6_solver.launch
$ roslaunch dawn_ik horti_solver.launch
$ roslaunch dawn_ik mick_solver.launch

# If you would like to use collision_ik...
# For lite6 + collision_ik:
$ roscd relaxed_ik_ros1/relaxed_ik_core/config/
$ cp lite6_settings.yaml settings.yaml # overwriting!
$ roslaunch dawn_ik lite6_solver_collision_ik.launch

# For horti + collision_ik:
$ roscd relaxed_ik_ros1/relaxed_ik_core/config/
$ cp horti_settings.yaml settings.yaml # overwriting!
$ roslaunch dawn_ik horti_solver_collision_ik.launch

# For mick + collision_ik:
$ roscd relaxed_ik_ros1/relaxed_ik_core/config/
$ cp mick_settings.yaml settings.yaml # overwriting!
$ roslaunch dawn_ik mick_solver_collision_ik.launch
```

### Experiments

Before doing the experiments make sure that:

- Generated code is for that robot, while using dawn_ik.
- `settings.yaml` is set for that robot, while using collision_ik.

For doing the experiments you can start everything **ALL-IN-ONE** line. Stop roscore and all other things. Available robots for experiments are `horti`, `lite6` and `mick`. Available solvers are `dawn_ik` and `collision_ik`.

```bash
# Example: Horti + DawnIK + test.txt
$ roscd dawn_ik/include/dawn_ik/robot_configuration
$ cp horti.h autogen_test.h
$ catkin build
$ roslaunch dawn_ik run_experiment.launch robot_name:=horti solver:=dawn_ik waypoints_file:=test endpoint_frame:=head_link_eef

# Example: Lite6 + CollisionIK + lower_y.txt
$ roscd relaxed_ik_ros1/relaxed_ik_core/config/
$ cp lite6_settings.yaml settings.yaml # overwriting!
$ roslaunch dawn_ik run_experiment.launch robot_name:=lite6 solver:=collision_ik waypoints_file:=lower_y endpoint_frame:=link_eef

# Example: Mick + CollisionIK + mid_y.txt + test.bag
$ roscd relaxed_ik_ros1/relaxed_ik_core/config/
$ cp mick_settings.yaml settings.yaml # overwriting!
$ roslaunch dawn_ik run_experiment.launch robot_name:=mick solver:=collision_ik waypoints_file:=mid_y trajectory_file:=test endpoint_frame:=head_link_eef
```

Waypoints are located in `waypoints` folder. Results are saved into the `results` folder. For analyzing and generating figures see `results/analyze_results.ipynb` notebook.

### TODO

- Add number of repetitions for the waypoints.

### Footnotes

- [1] [Ceres Solver](http://ceres-solver.org/) is heavily used in this project so we named this project similar to [how Ceres Solver is named](http://ceres-solver.org/#f1). [Dawn](https://solarsystem.nasa.gov/missions/dawn/overview/) is the spacecraft launched in 2007 by NASA, reached to Ceres in 2015 and acquired the dwarf planet's information of global shape, mean density, surface morphology, mineralogy, etc. by the middle of 2016. 

