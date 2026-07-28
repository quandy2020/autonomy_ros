# 左右臂标识参数，左臂=0x04，右臂=0x05
uint8 arm_side

# 运动类型，1=MOVE_J（关节空间运动）
uint8 motion_type

# 目标关节位置，7个关节目标坐标，单位根据实际定义，一般为度或弧度。float32为4字节
float32[7] q

#迪卡尔坐标
float32[6] x

#有效目标位置
uint8 qx_flag

# 速度比例，取值范围[0.05, 1]
float32 l_vscale

# 加速度比例，取值范围[0.05, 1]
float32 l_accscale
