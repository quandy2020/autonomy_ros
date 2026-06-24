# Topological / visibility graph for navigation and RViz visualization.
#
# Visibility graph convention (kind=KIND_VISIBILITY):
#   - nodes: obstacle polygon vertices + optional start/goal (use GraphNode.flags)
#   - edges: line-of-sight segments, TYPE_UNDIRECTED, LINK_SHAPE_LINE, weight=distance
#   - faces: leave empty
#
# Topological navmesh convention (kind=KIND_TOPOLOGICAL):
#   - nodes: region centers; edges: adjacency; faces: filled regions (optional)

uint8 KIND_GENERIC=0
uint8 KIND_TOPOLOGICAL=1
uint8 KIND_VISIBILITY=2

std_msgs/Header header
GraphNode[] nodes
GraphEdge[] edges
GraphFace[] faces
uint8 kind
geometry_msgs/Pose origin
