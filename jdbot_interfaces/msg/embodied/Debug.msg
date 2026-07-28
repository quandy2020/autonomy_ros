# 左右臂标识，左臂=0，右臂=1
uint8 arm_side

# 机器人运行状态
uint8 arm_status

float32[7] q_cmd
float32[7] dq_cmd
float32[7] torq_cmd
float32[6] x_cmd
float32[6] xd_cmd
float32[7] mposi_cmd
float32[7] mvel_cmd
float32[7] mtorq_cmd
float32[7] mposi_fb
float32[7] mvel_fb
float32[7] mtorq_fb