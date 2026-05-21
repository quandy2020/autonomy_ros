# Periodic task snapshot (topic: /autonomy/status, ~5 Hz).

std_msgs/Header header
string task_id                 # Client id from current action goal
string tour_id                 # GuidedTour tour_id when applicable
TaskType task_type
TaskState task_state
bool paused                    # True after PauseTask
string description             # e.g. running, waiting_for_continue, canceled
float32 progress               # Normalized 0.0~1.0 for multi-waypoint/tour tasks
Error error                    # Active error; NONE when healthy
uint32 current_waypoint_index  # NavigateThrough / GuidedTour index
string current_exhibit_id
string current_narration_id    # AV/TTS content id at current exhibit
geometry_msgs/PoseStamped current_pose
BatteryStatus battery
