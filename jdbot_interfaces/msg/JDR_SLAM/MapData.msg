# SLAM 的地图数据信息

bool global_change # true: global change, false: local change

## default : base frame
# pose_id.size() == poses.size() == node_size == ground_points.size() == obstacle_points.size()

KeyFrame[] keyframes

# link_type.size() == from_id.size() == to_id.size() == transform.size() == link_covariances.size() / 36
int8[] link_type  # 0: odometry, 1: loop closure
int32[] from_id
int32[] to_id
geometry_msgs/Pose[] transform
float64[] link_covariances # length must be 36 * link_type.size(), order is rpy xyz