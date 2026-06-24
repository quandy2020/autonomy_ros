# One undirected or directed edge between two GraphNode ids.
#
# Visibility graph edges typically use:
#   type=TYPE_UNDIRECTED, link_shape=LINK_SHAPE_LINE, valid=true,
#   path=[] (straight segment from node positions), weight=Euclidean distance.

uint8 TYPE_UNDIRECTED=0
uint8 TYPE_DIRECTED=1
uint8 TYPE_BIDIRECTIONAL=2
uint8 TYPE_VIRTUAL=3

uint8 LINK_SHAPE_LINE=0
uint8 LINK_SHAPE_TRIANGLE=1
uint8 LINK_SHAPE_RECTANGLE=2
uint8 LINK_SHAPE_POLYGON=3

uint32 from_id
uint32 to_id
uint8 type
uint8 link_shape
float64 weight
std_msgs/ColorRGBA color
uint32 id
bool valid
geometry_msgs/Point[] path
