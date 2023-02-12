# Salih Marangoz Thesis

### [===> Thesis Notes](thesis/THESIS_NOTES.md) (meetings, etc.)

### [===> My Notes](thesis/MY_NOTES.md)



## Installation

- http://ceres-solver.org/installation.html (2.x.x)

- https://github.com/xArm-Developer/xarm_ros (master)

## Running

```bash
# $ roslaunch horti_moveit_config demo_gazebo.launch
$ roslaunch salih_marangoz_thesis ceres_ik_sim.launch
$ roslaunch salih_marangoz_thesis ceres_ik_node.launch
```



## ToDo:

- **[EIGEN_NO_DEBUG](https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html)**: disables Eigen's assertions if defined. Not defined by default, unless the `NDEBUG` macro is defined (this is a standard C++ macro which disables all asserts).
