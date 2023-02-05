

```bash
$ git clone git@github.com:ceres-solver/ceres-solver.git
$ cd ceres-solver
$ mkdir build
$ cd build
$ cmake .. -DUSE_CUDA=0
$ make -j8
$ sudo make install # UNINSTALL: sudo make uninstall

$ roslaunch xarm_gazebo xarm7_beside_table.launch
$ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
```







----------





# My Personal Notes

- idea (1):
  - camera arm finds a fruit.
  - path from grasper to the fruit is calculated.
  - points far from this path with a fixed distance are sampled. points are connected in a graph. points should be directly connected to the closest path point.
  - shortest path on this graph for the camera arm. 
  - if the path doesn't found the same method can be applied by switching grasper and camera arm. camera arm first computes a path near to the object.
- idea (2):
  - so I am basically controlling a dual robot system with closed-loop kinematic chain, but with a relaxed linear joint between two robot endpoints. but URDF and Moveit don't support closed kinematics.
- idea (3)
  - camera arm finds a fruit
  - path from grasper to the fruit it calculated.
  - camera arm follows the grasper with visual servoing. this may be a bit problematic. how we will know that what we are look at a grasper? qr code? robot model point cloud discarding?


## Questions

- RRT-Connect?
- Does visual servoing always need continuous object tracking?
- whole-body-controller? do we need to move trolley or something else? or is this for following.
- Isaac Sim crashes a lot. 
- how to control robot arms with moveit? How to start Moveit? It doesn't work.
- do we need to observe other arms there is enough free space?
- ![Conventional+PBVS+Endpoint+open-loop+(EOL)_](assets/Conventional+PBVS+Endpoint+open-loop+(EOL)_.jpg)

## Related Works

- SYSTEM OVERVIEW. Leveraging Deep Learning and RGB-D Cameras for Cooperative Apple-Picking Robot Arms - [paper](papers/aim.201901125.pdf)
  - two robot arms, rgbd, each arm has cameras
  - Void Space Planning? nodes represent fruits and robot endpoint positions (only for grasper). edges represent straight line connection between fruit/robot or robot/robot nodes. shorted path on the graph is done bu Dijkstra.
  - search arm puts fruit nodes. grasp arm puts pose nodes and connects robot/fruit if fruit is seen from the grasper.
  - used RRT-Connect. 
  - **Thoughts:** in my case there will be two grasper arms but without cameras. so camera arm can put nodes and connect them to fruits. graspers can follow the graph and meanwhile camera arm can observe graspers.
- Occlusion-Free Visual Servoing for the Shared Autonomy Teleoperation of Dual-Arm Robots - [paper](papers/LRA.2018.2792143 (1).pdf)
  - two robot arms. user controls grasper arm, camera arm is autonomous doing visual servoing. 
  - **Checkref:** `5`, `8`, `14`
  - **Thoughts:** in my case user will not control any arms. so the system will know where the grasper will move. 


- Towards Active Robotic Vision in Agriculture: A Deep Learning Approach to Visual Servoing in Occluded and Unstructured Protected Cropping Environments - [paper](papers/1908.01885 (1).pdf)
  - todo
- REAL-TIME VISUAL SERVOING - [paper](papers/cucs-035-90 (1).pdf)

  - todo


## Emails

- I have a possible master thesis topic. The rough idea of the thesis is as follows: We have an arm that has grasped an object and another arm for sensing. We need to keep moving the second arm in a real time manner to ensure that the grasped object is in view of the camera to enable further manipulation actions. 
- The motivation comes from the Phenorob project I am working on. In it, we will have a dual arm manipulation system and a third arm (the middle arm) for perception. The dual arm system is used for harvesting and other horticulture actions. One arm will grasp the fruit, the other will try to cut the peduncle. In such a scenario, the middle arm needs to keep the peduncle in sight for providing feedback to the cutting arm. This if course a multi year project. In our case, we don't want to do visual servoing per se as the camera is mounted on a different arm. 
