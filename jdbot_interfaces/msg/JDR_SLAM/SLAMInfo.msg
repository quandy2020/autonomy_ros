
std_msgs/Header header

bool loop_closure_detected

#before_corrected_loop_signal: true if the loop closure is detected before correction
bool before_corrected_loop_signal

# type=1. loop closure with current map; type=2. loop closure with previous map
int8 loop_closure_type

# id of big change
int8 big_change_id

# the Transformation from old map to current map, that means a point in the new map
# satisfies P_new = T_new_old * P_old
geometry_msgs/Pose t_new_old
