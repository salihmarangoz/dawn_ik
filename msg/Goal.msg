# This message expresses TODO

int64 ee_mode # 0 for ee_point, 1 for ee_line

# Set ee_mode to 0
Constraint ee_point_x
Constraint ee_point_y
Constraint ee_point_z
Constraint ee_point_distance # Use this to set euclidean distance constraint. Combining this constraint with boundaries and target constraints may result stuck solutions.

# Set ee_mode to 1
float64 ee_line_x1
float64 ee_line_y1
float64 ee_line_z1
bool ee_line_is_p1_bounded
float64 ee_line_x2
float64 ee_line_y2
float64 ee_line_z2
bool ee_line_is_p2_bounded
Constraint ee_line_distance # use positive upper boundary to define a capsule

# Global rotations of eef
Constraint ee_roll
Constraint ee_pitch
Constraint ee_yaw

# Used to define a target point. Doesn't affect anything until other target_* constraints are set.
float64 target_point_x
float64 target_point_y
float64 target_point_z

# Usually the value is set to 0. Use boundaries for limiting valid viewpoints
Constraint target_offset_roll
Constraint target_offset_pitch
Constraint target_offset_yaw

# min/max distance to the target
Constraint target_distance