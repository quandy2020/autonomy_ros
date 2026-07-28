# 左右臂标识参数，左臂=0x04，右臂=0x05
uint8 hand_side

uint8 posi_cmd # 0~255(120mm)
uint8 vel_cmd  # 0~255
uint8 force_cmd #0~255(30N)
uint8 acc_cmd #0~255
uint8 dec_cmd #0~255