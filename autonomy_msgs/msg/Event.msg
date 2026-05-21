# Edge-triggered notifications for App / AV / cloud (topic: /autonomy/events).

uint8 ARRIVED_AT_EXHIBIT=1     # Play narration_id; robot at exhibit
uint8 LEFT_EXHIBIT=2
uint8 TOUR_STARTED=3
uint8 TOUR_COMPLETED=4
uint8 TOUR_PAUSED=5
uint8 TOUR_RESUMED=6
uint8 LOW_BATTERY=7
uint8 EMERGENCY_STOP=8
uint8 DOCK_STARTED=9
uint8 DOCK_COMPLETED=10
uint8 FOLLOW_TARGET_LOST=11
uint8 TASK_PREEMPTED=12        # Higher-priority task replaced active task

std_msgs/Header header
uint8 event_type
string tour_id
string task_id
string exhibit_id
string narration_id            # Often set on ARRIVED_AT_EXHIBIT
string message                 # Optional detail (e.g. preempted ids)
TaskStatus status              # Snapshot at event time
