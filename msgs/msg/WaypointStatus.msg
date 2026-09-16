# Per-waypoint status for NavigateThroughPoses (Nav2-compatible)
uint8 PENDING=0
uint8 COMPLETED=1
uint8 FAILED=2
uint8 SKIPPED=3

uint16 waypoint_index
geometry_msgs/PoseStamped waypoint_pose
uint8 waypoint_status
uint16 error_code
string error_msg
