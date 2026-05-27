# Edge-triggered task notifications (topic: /autonomy/events).

uint8 TASK_PREEMPTED=1
uint8 TASK_PAUSED=2
uint8 TASK_RESUMED=3
uint8 EMERGENCY_STOP=4

std_msgs/Header header
uint8 event_type
string task_id
string message
TaskStatus status
