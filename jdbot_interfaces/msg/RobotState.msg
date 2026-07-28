# jdbot_interfaces/msg/robotState.msg

# Robot State Message

std_msgs/Header header

# Basic status
bool emergency            # 急停
bool left_bumper          # 左碰撞
bool right_bumper         # 右碰撞
bool front_bumper         # 前碰撞
bool leftback_bumper      # 左后碰撞
bool rightback_bumper     # 右后碰撞
bool fall_status          # 机器跌倒

# bumper value
uint8 left_bumper_adc     # 左碰撞传感器ADC值
uint8 right_bumper_adc    # 右碰撞传感器ADC值
uint8 front_bumper_adc    # 前碰撞传感器ADC值
uint8 leftback_bumper_adc # 左后碰撞传感器ADC值
uint8 rightback_bumper_adc # 右后碰撞传感器ADC值

# Error codes
uint8 hip_err_l           # 左髋故障码
uint8 hip_err_r           # 右髋故障码
uint8 knee_err_l          # 左膝故障码
uint8 knee_err_r          # 右膝故障码         
uint8 wheel_err_l         # 左轮故障码          
uint8 wheel_err_r         # 右轮故障码

#Waist status
uint8 waist_status         # 腰部状态
uint8 waist_mot_err         # 腰部故障码

# Chassis information
uint32 chassis_err        # 底盘错误码
uint8 chassis_status      # 底盘状态
uint8 chassis_mode        # 底盘模式

# Camera and arm status
uint8 camera_status       # 云台状态
uint16 camera_err         # 云台故障码
uint8 arm_status_l        # 左胳膊状态
uint16 arm_err_l          # 左胳膊故障码
uint8 arm_status_r        # 右胳膊状态
uint16 arm_err_r          # 右胳膊故障码

# Enums for status (camera_status, arm_status_l, arm_status_r)
uint8 STATUS_INIT = 0
uint8 STATUS_IDLE = 1
uint8 STATUS_ERR = 2
uint8 STATUS_DISABLE = 3

# Enum for chassis_mode
uint8 MODE_BW_BASE_WHEEL_BALANCE = 1  # 平衡模式
uint8 MODE_BW_BASE_WHEEL_STATIC = 2   # 静态模式
uint8 MODE_BW_BASE_WHEEL_FOLD = 3     # 阻尼折叠模式

# Enum for chassis_status
uint8 WHEEL_CTR_STATUS_INIT = 0
uint8 WHEEL_CTR_STATUS_SPD = 1
uint8 WHEEL_CTR_STATUS_ERR = 2
uint8 WHEEL_CTR_STATUS_DISABLE = 3

# Enum for chassis_err
uint32 WHEEL_CTR_ERR_OK = 0x00000000
uint32 WHEEL_CTR_ERR_HOST_TIMEOUT = 0x00000001
uint32 WHEEL_CTR_ERR_IMU = 0x00000004
uint32 WHEEL_CTR_ERR_LOSE_HEART1 = 0x00000010
uint32 WHEEL_CTR_ERR_LOSE_SPD1 = 0x00000020
uint32 WHEEL_CTR_ERR_MOT1 = 0x00000040
uint32 WHEEL_CTR_ERR_COMM1 = 0x00000080
uint32 WHEEL_CTR_ERR_LOSE_HEART2 = 0x00000100
uint32 WHEEL_CTR_ERR_LOSE_SPD2 = 0x00000200
uint32 WHEEL_CTR_ERR_MOT2 = 0x00000400
uint32 WHEEL_CTR_ERR_COMM2 = 0x00000800
uint32 WHEEL_CTR_ERR_LOSE_HEART3 = 0x00001000
uint32 WHEEL_CTR_ERR_LOSE_SPD3 = 0x00002000
uint32 WHEEL_CTR_ERR_MOT3 = 0x00004000
uint32 WHEEL_CTR_ERR_COMM3 = 0x00008000
uint32 WHEEL_CTR_ERR_LOSE_HEART4 = 0x00010000
uint32 WHEEL_CTR_ERR_LOSE_SPD4 = 0x00020000
uint32 WHEEL_CTR_ERR_MOT4 = 0x00040000
uint32 WHEEL_CTR_ERR_COMM4 = 0x00080000
uint32 WHEEL_CTR_ERR_LOSE_HEART5 = 0x00100000
uint32 WHEEL_CTR_ERR_LOSE_SPD5 = 0x00200000
uint32 WHEEL_CTR_ERR_MOT5 = 0x00400000
uint32 WHEEL_CTR_ERR_COMM5 = 0x00800000
uint32 WHEEL_CTR_ERR_LOSE_HEART6 = 0x01000000
uint32 WHEEL_CTR_ERR_LOSE_SPD6 = 0x02000000
uint32 WHEEL_CTR_ERR_MOT6 = 0x04000000
uint32 WHEEL_CTR_ERR_COMM6 = 0x08000000
uint32 WHEEL_CTR_ERR_SYS = 0x10000000

# Enum for error bits (camera_err, arm_err_l, arm_err_r)
uint16 ERR_VOLTAGE = 0x01      # bit0: 电压故障
uint16 ERR_CURRENT = 0x02      # bit1: 电流故障
uint16 ERR_TEMPERATURE = 0x04  # bit2: 温度故障
uint16 ERR_ENCODER = 0x08      # bit3: 编码器故障
uint16 ERR_HARDWARE = 0x40     # bit6: 硬件故障
uint16 ERR_SOFTWARE = 0x80     # bit7: 软件故障

# RS02 关节电机错误码 (hip_err_l, hip_err_r, knee_err_l, knee_err_r)
uint8 RS02_ERR_UNDER_VOLTAGE = 0x01     # bit0: 欠压
uint8 RS02_ERR_OVER_CURRENT = 0x02      # bit1: 过流
uint8 RS02_ERR_OVER_TEMPERATURE = 0x04  # bit2: 过温
uint8 RS02_ERR_MAGNETIC_ENCODER = 0x08  # bit3: 磁编码器故障
uint8 RS02_ERR_HALL_ENCODER = 0x10      # bit4: 霍尔编码器故障
uint8 RS02_ERR_UNDEFINED = 0x20         # bit5: 未标注

# 本末1505B轮电机错误码 (wheel_err_l, wheel_err_r)
uint8 WHEEL_ERR_NONE = 0x00              # 无故障
uint8 WHEEL_ERR_UNDER_VOLTAGE_2 = 0x02   # 欠压 2 (母线电压 < 18V)
uint8 WHEEL_ERR_UNDER_VOLTAGE_1 = 0x01   # 欠压 1 (18V < 母线电压 < 20V)
uint8 WHEEL_ERR_OVER_VOLTAGE = 0x03      # 过压 (母线电压 > 36V)
uint8 WHEEL_ERR_OVER_CURRENT = 0x0A      # 过流 (默认: 母线电流 > 35A)
uint8 WHEEL_ERR_OVER_TEMP_1 = 0x20       # 过温 1 (电机绕组温度 > 80°C)
uint8 WHEEL_ERR_OVER_TEMP_2 = 0x1F       # 过温 2 (电机绕组温度 > 110°C)
uint8 WHEEL_ERR_SAMPLING = 0x29          # 采样电阻故障
uint8 WHEEL_ERR_POSITION_SENSOR = 0x2A   # 位置传感器自身故障
uint8 WHEEL_ERR_POSITION_SENSOR_SIGNAL = 0x2B  # 位置传感器信号被干扰
uint8 WHEEL_ERR_TEMP_SENSOR = 0x2D       # 温度传感器超出量程
uint8 WHEEL_ERR_COMMUNICATION = 0x3C     # 通信超时(默认无保护，需用户自行开启)
uint8 WHEEL_ERR_STALL = 0x62             # 堵转(默认:电流 > 5A 且 转速为 0)

# Note: camera_err, arm_err_l, arm_err_r 使用位操作来组合多个错误
# camera_err: 前八位是YAW轴电机（ID：11），后八位是PITCH电机（ID：12）
# arm_err_l: 前八位是左前电机（ID：7），后八位是左后电机（ID：8）
# arm_err_r: 前八位是右前电机（ID：9），后八位是右后电机（ID：10）
#例如 camera_err 为 0x0002，注意为十六进制，转化为二进制为：00000000 00000010
#对应故障码，故障原因为PITCH电机（ID：12）电流故障，可能为长时间堵转导致