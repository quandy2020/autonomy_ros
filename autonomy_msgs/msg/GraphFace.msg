# Filled graph facet (triangle / rectangle / polygon region).

uint8 SHAPE_TRIANGLE=0
uint8 SHAPE_RECTANGLE=1
uint8 SHAPE_POLYGON=2

uint8 shape
uint32[] node_ids
geometry_msgs/Point[] points
std_msgs/ColorRGBA color
float32 alpha
