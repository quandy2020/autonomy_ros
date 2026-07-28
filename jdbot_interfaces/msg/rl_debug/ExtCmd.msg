# 命令信息
int32 current_mode    # 当前控制模式 (ControlMode enum)
float32 vx_in         # 传入的线速度信息
float32 vx_processed  # 运控处理后的vx数据
float32 wz_in         # 传入的wz角速度信息
float32 wz_processed  # 运控处理后的角速度数据
float32 height_in     # 传入的高度指令(m)
float32 height_processed  #运控处理后的高度指令（m）
float32 vx_offset       #线速度vx的补偿量
float32 wz_offset       #角速度wz的补偿量
