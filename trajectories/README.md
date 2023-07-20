# Recording Trajectories for Replay

This is for controlling other arms (not controller by DawnIK, CollisionIK, etc.). The main script used to replay trajectories is `scripts/experiments/moveit_replay_trajectory.py`

## Recording

Start the simulator, then start recording:

```bash
$ roscd dawn_ik/trajectories

# For horti:
$ rosbag record /arm_left_controller/follow_joint_trajectory/goal /arm_right_controller/follow_joint_trajectory/goal

# For mick:
$ rosbag record /other_controller/follow_joint_trajectory/goal
```

## Playing

For different use cases and parameters, check the script.

```bash
$ rosrun dawn_ik moveit_replay_trajectory.py _bag_filename:="$(rospack find dawn_ik)/trajectories/2023-07-20-03-21-24.bag"
```

