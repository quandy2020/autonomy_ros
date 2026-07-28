# SLAM 的关键帧数据信息

## default : base frame
int32 poses_id
int32 map_id
float64 timestamp
geometry_msgs/Pose poses
geometry_msgs/Pose poses_camera

# length must be 3 * point_num
float32[] ground_points
float32[] obstacle_points

#features
float32[] key_points
float32[] points_3ds
int8[] descriptors
