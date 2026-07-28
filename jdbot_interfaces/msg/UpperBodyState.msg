# 上身状态信息

float32 camera_yaw_rad
float32 camera_pitch_rad
uint8   camera_state
uint16  camera_err_code
float32 left_arm_roll_rad
float32 left_arm_pitch_rad
uint8   left_arm_state
uint16  left_arm_err_code
float32 right_arm_roll_rad
float32 right_arm_pitch_rad
uint8   right_arm_state
uint16  right_arm_err_code
uint8   left_process_bar
uint8   right_process_bar

# 状态枚举值
uint16 ERROR_OK = 0x0701            # 正常
uint16 HEAD_CTR_ERR_OK = 0x0800     # 空闲停止
uint16 HEAD_CTR_ERR_FRAME = 0x0801  # 数据校验失败
uint16 HEAD_CTR_ERR_MOVE = 0x0802   # 运动中
uint16 HEAD_CTR_ERR_ODOM = 0x0803   # 编码器数据出错
uint16 HEAD_CTR_ERR_ZERO = 0x0805     # 头部云台校准零度中
uint16 HEAD_CTR_ERR_TIMEOUT = 0x0806 # 电机堵转或机械错位，长时间运动，无法运动到指定角度，需要手动清零
