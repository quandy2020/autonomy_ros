# event type

uint16 EVENT_ROBOT_FALLDOWN = 1001
uint16 EVENT_ROBOT_POWEROFF= 1002

uint16 EVENT_PLANNING_GLARE = 6001
uint16 EVENT_PLANNING_LOW_LIGHT = 6002
uint16 EVENT_PLANNING_BREAK_FREE = 6003
uint16 EVENT_PLANNING_STUCK_CONDITION = 6004
uint16 EVENT_PLANNING_COLLISION = 6005
uint16 EVENT_PLANNING_EMERGENCY_STOP = 6006
uint16 EVENT_SLAM_LOST_POSITION = 6008 
uint16 EVENT_PLANNING_OBSTACLE_AVOIDING = 6016

uint16 EVENT_AUTONOMOUS_SIT = 8101

uint16 EVENT_ALGO_FALLDOWN = 9001
uint16 EVENT_ALGO_WATER = 9002
uint16 EVENT_ALGO_WAVEHAND = 9003
uint16 EVENT_ALGO_GESTURE = 9004

uint16 event


# Event-specific data payloads.
# Only the payload corresponding to the 'event' field will be populated.

FallDownEvent falldown_event
PowerOffEvent poweroff_event
WaterEvent water_event
WaveHandEvent wavehand_event
GestureEvent gesture_event
