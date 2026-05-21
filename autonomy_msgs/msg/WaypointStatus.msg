# Per-waypoint result (NavigateThrough / GuidedTour exhibit_statuses).

uint8 PENDING=0
uint8 COMPLETED=1
uint8 SKIPPED=2
uint8 FAILED=3

uint8 waypoint_status
uint32 waypoint_index
geometry_msgs/PoseStamped waypoint_pose
uint16 error_code              # autonomy_msgs/Error constant when FAILED
string error_msg
