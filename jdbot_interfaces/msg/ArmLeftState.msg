# 左胳膊状态信息

# Yaw 角度（弧度），范围：±150
float32 yaw_rad

# Pitch 角度（弧度），范围：±56
float32 pitch_rad

# 当前 Yaw 角速度（弧度/秒）
float32 current_yaw_speed

# 当前 Pitch 角速度（弧度/秒）
float32 current_pitch_speed

# 左胳膊状态
uint16 state

# 状态枚举值
uint16 ERROR_OK = 0x0701            # 正常
uint16 HEAD_CTR_ERR_OK = 0x0800     # 空闲停止
uint16 HEAD_CTR_ERR_FRAME = 0x0801  # 数据校验失败
uint16 HEAD_CTR_ERR_MOVE = 0x0802   # 运动中
uint16 HEAD_CTR_ERR_ODOM = 0x0803   # 编码器数据出错
uint16 HEAD_CTR_ERR_ZERO = 0x0805     # 左胳膊校准零度中
uint16 HEAD_CTR_ERR_TIMEOUT = 0x0806 # 电机堵转或机械错位，长时间运动，无法运动到指定角度，需要手动清零
