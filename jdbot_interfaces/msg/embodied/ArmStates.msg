# 左右臂标识，左臂=0，右臂=1
uint8 arm_side

# 机器人运行状态
uint8 arm_status

# 错误代码
uint8 error_code

# 7个关节的当前位置（弧度）
float32[7] joint_pos

# 7个关节的当前速度（弧度/秒）
float32[7] joint_vel

# 7个关节的当前力矩（N·m）
float32[7] joint_torque

# TCP当前位置和姿态（x, y, z, roll, pitch, yaw）
float32[6] tcp_pose

# 当前TCP速度（线速度/角速度）
float32[6] tcp_twist