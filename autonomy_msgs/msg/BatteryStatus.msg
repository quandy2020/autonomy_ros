# Battery report (topic: /autonomy/battery).

std_msgs/Header header
float32 percentage 0.0         # State of charge 0~100
float32 voltage 0.0            # Optional; 0 if unknown
bool charging false
bool low_battery false         # True when percentage < low_battery_threshold
float32 low_battery_threshold 20.0
bool critical_battery false
float32 critical_battery_threshold 10.0
