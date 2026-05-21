# Follow mode target specification (Follow action).

bool use_target_id false       # Track named target (requires perception stack)
string target_id

bool use_target_pose false     # Track a fixed or externally updated pose
geometry_msgs/PoseStamped target_pose

float32 follow_distance 1.0    # Stand-off distance when pose reached (m)
float32 max_linear_speed 0.5   # Cap while following (m/s)
string target_frame map        # Frame for target_pose
