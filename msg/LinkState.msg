# This message expresses syncronized joint and link state in a single message

Header header

# Global link transformations. Ordered as xyz translations and wxyz quaternions; tx0,ty0,tz0,rw0,rx0,ry0,rz0,tx1,ty1,tz1,rw1,rx1,ry1,rz1,...
# This represents the link origins. Similar to gazebo_msgs/LinkState
# Length: 7*num_links
float64[] transformations