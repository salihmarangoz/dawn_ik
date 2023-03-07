# This message expresses syncronized joint and link state in a single message

# frame_id may be empty. check robot's world joint in this case.
Header header

# Joint state. Same header with the above one.
sensor_msgs/JointState joint_state

# Global link transformations. Ordered as xyz translations and wxyz quaternions; tx0,ty0,tz0,rw0,rx0,ry0,rz0,tx1,ty1,tz1,rw1,rx1,ry1,rz1,...
# This represents the link origins. Similar to gazebo_msgs/LinkState
float64[] link_state

# Collision pairs with their indexes. These are not links but predefined collision objects in the robot configuration header.
uint32[] collision_pair0
uint32[] collision_pair1