# JointType.msg - 关节类型枚举（适配未来扩展）
uint8 JOINT_UNKNOWN = 0    # 未知关节（默认值）
uint8 JOINT_HEAD = 3       # 头部关节（与原 gRPC 枚举值对齐）
uint8 JOINT_LEFT_ARM = 4   # 左臂关节（与原 gRPC 枚举值对齐）
uint8 JOINT_RIGHT_ARM = 5  # 右臂关节（与原 gRPC 枚举值对齐）
# 预留扩展位：未来新增关节直接在此处添加（如腿部、腰部）
uint8 JOINT_RESERVED_1 = 6 
uint8 JOINT_RESERVED_2 = 7 
uint8 JOINT_RESERVED_3 = 8 

uint8 JOINT_RL_CONTROL = 100 # 通过强化学习控制关节

uint8 type  # 实际使用的关节类型（取值为上述枚举）