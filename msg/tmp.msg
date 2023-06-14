###################################################################################################
# MODE_(?): CONVERGE TO PRIOR
###################################################################################################

# Set to MODE_1 and leave constraint weights as zero.
# Joints may move to the their preferred positions while avoiding collisions and respecting other objectives.

###################################################################################################
# MODE_1: ENDPOINT POSITION CONTROL W.R.T. A POINT
###################################################################################################

# Constraints define:
# - point;                         with 3 variable(s) hard-bounded
# - axis aligned bounding volume;  with 3 variable(s) soft-bounded
# - axis aligned line;             with 2 variable(s) hard-bounded + 1 variable(s) relaxed
# - axis aligned segment;          with 2 variable(s) hard-bounded + 1 variable(s) soft-bounded
# - axis aligned plane;            with 1 variable(s) hard-bounded + 2 variable(s) relaxed
# - axis aligned 2d rectangle;     with 2 variable(s) hard-bounded + 1 variable(s) relaxed

# The parameter "m1_distance" adds extra distance tolerance to the defined shape. For example;
# Constraints define:
# - a sphere;           with 3 variable(s) hard-bounded + m1_distance>0

# If you would like to define more than axis aligned shapes check MODE_1.

Constraint m1_x
Constraint m1_y
Constraint m1_z
Constraint m1_distance # euclidean distance tolerance



###################################################################################################
# MODE_2: ENDPOINT POSITION CONTROL W.R.T. A LINE
###################################################################################################

# Constraints define:
# - line;         with p1 and p2 unbounded
# - ray;          with one bounded and one unbounded
# - line segment; with p1 and p2 bounded

# The parameter "m2_distance" adds extra distance tolerance to the defined shape. For example;
# Constraints define:
# - infinite height capsule/cylinder;        with p1 and p2 unbounded + m2_distance>0
# - infinite height capsule with a start;    with one bounded and one unbounded + m2_distance>0
# - capsule;                                 with p1 and p2 bounded + m2_distance>0

# This constraint also defines a point in space. If point1==point2 please use MODE_1 instead.

# Point-1
float64    m2_p1x
float64    m2_p1y
float64    m2_p1z
bool       m2_p1_bounded

# Point-2
float64    m2_p2x
float64    m2_p2y
float64    m2_p2z
bool       m2_p2_bounded

# Distance tolerance to the defined shape. Valid values are m2_distance>=0
Constraint m2_distance # Recommended to always set m2_distance.lower_tolerance = 0




###################################################################################################
# MODE_3: ENDPOINT ORIENTATION CONTROL IN WORLD FRAME
###################################################################################################

# Constraints define:
# - keep camera upright;    with bounded m3_roll + bounded rp1_pitch
# - fixed direction;        with bounded m3_roll + bounded m3_pitch + bounded m3_yaw

Constraint m3_roll
Constraint m3_pitch
Constraint m3_yaw

###################################################################################################
# MODE_4: ENDPOINT POSITION AND ORIENTATION CONTROL W.R.T. TARGET
###################################################################################################

# Constraints define:
# - keep distance to the target;        with soft or hard bounded m4_distance
# - look at target;                     with hard-bounded m4_offset_roll, m4_offset_pitch, m4_offset_yaw
# - keep target in view;                with soft-bounded m4_offset_roll, m4_offset_pitch, m4_offset_yaw
# - look at target and keep distance;   with soft-bounded m4_distance + soft-bounded m4_offset_roll, m4_offset_pitch, m4_offset_yaw

# Always set the target position!
float64    m4_point_x
float64    m4_point_y
float64    m4_point_z

Constraint m4_distance # This is the only parameter affecting final position heavily in this mode.

# Angle offset to the target in the endpoint frame. To keep the target in camera image, appropriate values may be needed to set w.r.t. camera parameters
# Set values to 0 to keep the image on center and soft-bound to minimize energy use while tracking a moving target.
# Always set boundaries otherwise, 
Constraint m4_offset_roll
Constraint m4_offset_pitch
Constraint m4_offset_yaw

###################################################################################################
# MODE_5: ENDPOINT POSITION CONSTRAINING WITH CONVEX HULL
###################################################################################################

# Constraints define:
# - convex hull working environment;   with m5_convex_hull and optionally adding more tolerance with m5_distance

# NOTE: To restrict whole robot body, use external collisions input of the controller

sensor_msgs/PointCloud m5_convex_hull
float64 m5_distance # valid values are; m5_distance>=0