# rfid找物品功能相关消息定义

# 标签mac值
string item_mac

# 天线ID  0 ～ 1 （天线数量 2）
uint32 ant_id

# 角度值，0 ～ 180 ，单位 度
float32 angle

# 信号强度，-128 ～ 0
float32 rssi

# 相位
float32 phase

# 天线与标签距离，单位 米
float32 distance

# E710状态
uint16 state

# 状态枚举值
uint16 RFID_OK = 0          # 正常
uint16 RFID_ERR_ANT = 1     # 天线连接异常
uint16 RFID_ERR_PORT = 2    # 串口异常
uint16 RFID_ERR_CHECKSUM = 4   # 串口帧校验错误
