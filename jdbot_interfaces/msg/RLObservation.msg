# header 包含时间戳 (stamp) 和 坐标系 (frame_id)
std_msgs/Header header

# ==============================
#           P0 核心观测
# ==============================

# --- 1. 关节速度 (Joint Velocities) ---
# 使用显式命名，避免数组索引混淆
float64 vel_hip_l      # 左髋关节速度 (rad/s)
float64 vel_hip_r      # 右髋关节速度 (rad/s)
float64 vel_knee_l     # 左膝关节速度 (rad/s)
float64 vel_knee_r     # 右膝关节速度 (rad/s)
float64 vel_wheel_l    # 左轮电机速度 (rad/s)
float64 vel_wheel_r    # 右轮电机速度 (rad/s)

# --- 2. 质心状态 (CoM Velocity - EKF/IMU Fused) ---
# 物理含义：机身质心在机身坐标系下的速度 (通常融合了 IMU)
float64 com_vel_linear_x   # m/s
float64 com_vel_yaw        # rad/s

# --- 3. 两轮中心速度 (Wheel Center Velocity - Diff Model) ---
# 物理含义：仅根据左右轮速和差分运动学模型推算的底盘中心速度
# 来源参考：原始 /odom 或轮速编码器计算
float64 diff_vel_linear_x  # m/s (vx)
float64 diff_vel_yaw       # rad/s (wz)

# ==============================
#           P1 辅助观测
# ==============================

# --- 4. 相机云台状态 (Camera Joints) ---
# 来源参考：/joint_states
float64 camera_yaw         # rad
float64 camera_pitch       # rad

# 状态位 (0=OK, 1=Error, etc.)
uint8 status_code
