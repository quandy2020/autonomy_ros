# One vertex in a Graph.

uint32 FLAG_NONE=0
uint32 FLAG_START=1
uint32 FLAG_GOAL=2
uint32 FLAG_OBSTACLE_VERTEX=4
uint32 FLAG_WAYPOINT=8

uint32 id
geometry_msgs/Point position
std_msgs/ColorRGBA color
float32 scale  # 0 = use RViz display default
uint32 flags
