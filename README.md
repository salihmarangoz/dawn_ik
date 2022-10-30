# Salih's Master Thesis with Rohit

[toc]

## Meetings

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
