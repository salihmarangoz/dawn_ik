# Salih's Master Thesis with Rohit

[toc]

## Workspace

```bash
# Gitlab
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/salih_marangoz_thesis.git
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/horti_model.git
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/gazebo_ros_link_attacher.git
$ git clone git@gitlab.igg.uni-bonn.de:phenorob/oc2/active_perception/capsicum_plant_detachable.git

# Github
$ git clone git@github.com:salihmarangoz/capsicum_superellipsoid_detector.git
$ git clone git@github.com:salihmarangoz/superellipsoid_msgs.git
$ git clone git@github.com:Eruvae/octomap_vpp.git
$ git clone git@github.com:Eruvae/octomap_vpp_rviz_plugin.git
$ sudo apt install libceres-dev
# xarm_ros: https://github.com/xArm-Developer/xarm_ros#4-getting-started-with-xarm_ros

# Other dependencies
$ cd catkin_ws/
$ rosdep install --from-paths src --ignore-src -r

# Not available but needed:
# trolley_description

# Extra. Dont need them now:
# voxblox: https://voxblox.readthedocs.io/en/latest/pages/Installation.html
```



## To-Do

Note: This section can be confusing. Please check the [Meetings](#meetings) section instead.

- [x] Prepare detachable capsicum plant models.
- [x] grasping plugin
- [ ] gripper and cutter gazebo models

- [ ] bioIK
  - [ ] LookAtGoal
  - [ ] minimal displacement goal, etc.
- [ ] maybe move to see while fruit is being cut but find the initial position with our method panoptic mapping and feed with via a basic pixel and depth based fake panoptic segmentation superellipsoid shape detector
- [ ] try wbc only for camera arm
- [ ] try move to see for camera arm

- [ ] very end of the thesis: 
  - [ ] arm priority:
    - head arm high priority
    - manipulator high p.
    - equal p.

## Meetings

Note: Some meeting notes may not be available.

### 23 Nov 2022

- Forked a repository from Github and added to https://gitlab.igg.uni-bonn.de/phenorob/oc2/active_perception/gazebo_ros_link_attacher with modifications for detachable capsicum fruit picking.
  - Gazebo is pretty messy about detaching joints while running. I have encountered more problems while attaching/detaching fruits and other objects. These segfaults are mostly related to bad implementation of the Detach() function. In a good implementation, I think it should be queuing detach call and run it before the physics engine runs, and even block it with a mutex.
  - Someone found a way to detach just before the physics engine runs: https://answers.gazebosim.org//question/12118/intermittent-segmentation-fault-possibly-by-custom-worldplugin-attaching-and-detaching-child/?answer=12251#post-id-12251
  - Open github issue: https://github.com/gazebosim/gazebo-classic/issues/1815
  - I have also added mutex and solved a problem causing segfault (can't delete added fixed_joint's from the vector. let it to become a minor memory leak to solve segfault.).
  - I hope there will be no more issues in the grasping side.
  - **Also**, I am planning to publish fruit poses using the same plugin if they are difficult to parse. Or, I can write a simple world plugin just to do it so we can use it somewhere else. `gazebo_link_pose_publisher`, etc.

### 16 Nov 2022

- I have tried plugging fruits with multiple joints for a realistic plant. It didn't worked as intended. It looks fine at the first sight until we try attaching the fruits to the robot head arm. 
- Maybe using a position controller for the arm in Gazebo is the mistake. Because position controller tries to position the parts of the arm to a certain location but it is physically not possible. This can also cause creating very high velocities/accelerations on other objects because of the forced collisions. If we can use a effort controller it may be more stable in the simulation. When the arm hits a wall it should stop and force of the arm shouldn't be enough to move the wall nor the arm. Because things go crazy in Gazebo with the position controller. **Or maybe**, I analyzed the problem wrongly and the problem is caused by the physics solver of Gazebo. I don't know...
- Making the plant body static wasn't a good idea because it is not possible to make individual links/models static when we load multiple models. So, I have decided to increase the weight of the plant body to a high value and modify its collision model so the there will be no collision on the leaves and the model will stay on the ground plane.

### 02 October 2022

- Created [capsicum_plant_detachable](https://gitlab.igg.uni-bonn.de/phenorob/oc2/active_perception/capsicum_plant_detachable) repository. Includes detachable fruits for 6 different plants.

### 26 October 2022

- Modified the plant model for manipulating fruits separately.

  - ~~Model: `assets/capsicum_plant_5_detachable` in this repo. **But don't use this model in anywhere because it has problems!**~~ (see 02 October 2022 meeting)

  - Demo Video: https://youtu.be/uZXLcmiAmCc

  - Model Editing Video (with 3DS Max): https://youtu.be/vt0vLy8touw

  - Example code:

    ```python
    import rospy
    from gazebo_msgs.msg import LinkState
    import numpy as np
    
    pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=2)
    rospy.init_node('test_node')
    r = rospy.Rate(100) # 10hz
    d = 0.0
    while not rospy.is_shutdown():
        m = LinkState()
        m.link_name = "capsicum_plant_5_detachable::link_capsicum_plant_5_fruit1"
        m.reference_frame = "capsicum_plant_5_detachable::link_capsicum_plant_5_body"
        m.pose.position.x = (1 - np.cos(d))/20
        pub.publish(m)
        r.sleep()
        d += 0.02
    ```

- With the method described above, the position of fruits **can't** be found. But it is possible to pass this information to the model:

  - Model Editing Video (with 3DS Max): https://youtu.be/Hul2IaU87Y8
  - The world position of the fruit can be found in `/gazebo/link_states`.
  - I have tested the model and it works 
  - Bounding box can be seen beautifully after this modification:

  ![ss1](assets/ss1.png)

- I think with these modifications the robot can easily detect the closest fruit and manipulate its position. So, should I continue and apply this to other models as well?
