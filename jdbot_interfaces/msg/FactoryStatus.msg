# 产测状态上报（王玉熙 2026-06-17 添加）
# 协议 0xF1, ctrl->host
# imu包接收统计，产测判断imu是否丢包
uint32 imu_pack_number
# 腿部电机接收包统计，产测判断是否丢包（n=0~5，共6个电机）
uint32[6] motor_packet_number