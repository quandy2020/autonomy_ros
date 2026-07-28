# RL Topic消息定义 - 层次化结构
# 标准ROS2头部
std_msgs/Header header

# 系统状态信息
SystemStatus system_status

# 机器人位姿信息
RobotPose robot_pose

# 关节组信息
JointGroup joints

# IMU传感器数据
ImuData imu

# EKF估计信息
EkfEstimate ekf_estimate

#cmd 命令信息
ExtCmd ext_cmd

# 网络推理信息
Inference network_inf
