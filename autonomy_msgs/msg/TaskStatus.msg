# Task snapshot (topic: /status, ~5 Hz).

std_msgs/Header header
string task_id
TaskType task_type
TaskState task_state
bool paused
string description
float32 progress
Error error
uint32 current_waypoint_index
geometry_msgs/PoseStamped current_pose
