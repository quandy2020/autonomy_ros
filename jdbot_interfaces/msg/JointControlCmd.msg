# JointControlCmd.msg - 关节控制指令（核心控制字段）
# 所有角度单位：弧度（对应角度值在注释中补充）
# 正负方向定义：见各关节注释

# 目标关节（指定控制头部/左臂/右臂）
JointType target_joint

# 当target_joint 为 JOINT_HEAD/JOINT_LEFT_ARM/JOINT_RIGHT_ARM时生效
# 目标角度参数（根据target_joint的类型，对应不同关节的pitch/yaw轴）
float32 pitch
float32 yaw

# 当target_joint 为 JOINT_RL_CONTROL时生效
# RL控制参数, more details about RLControlParam, see http://joyspace.jd.com/pages/al8ezTy3OFgpbvvm9MPU
RLControlType head_control_type
builtin_interfaces/Duration head_duration
RLControlType upper_control_type
builtin_interfaces/Duration upper_duration
RLControlType lower_control_type
builtin_interfaces/Duration lower_duration

# ------------------------------
# 各关节范围限制说明（v0.4.0硬件）
# http://xingyun.jd.com/codingRoot/jd-robot-workspace/system-archbase/blob/develop/modules/jdbot_control/motion_driver_tock/param/chassis_param_bao.yaml#L55
# ------------------------------
# 1. 头部关节（target_joint.type=JOINT_HEAD）
#   - pitch（俯仰）：
#     范围：[-0.525, 0.314] rad （对应-30°~18°）
#     方向：负值=向下转动，正值=向上转动
#   - yaw（旋转）：
#     范围：[-1.57, 1.57] rad （对应-90°~90°）
#     方向：负值=顺时针旋转，正值=逆时针旋转

# 2. 左臂关节（target_joint.type=JOINT_LEFT_ARM）
#   - pitch（抬起）：
#     范围：[-0.525, 2.66] rad （对应-30°~150°，硬件最大支持~2.66rad）
#     方向：负值=向下转动，正值=向上转动
#   - yaw（翻转）：
#     场景1：pitch < 2.4rad 时，范围[0.0, 0.525] rad（0°~30°）
#     场景2：pitch ≥ 2.4rad 时，范围[-0.26, 0.525] rad（-15°~30°）
#     方向：正值=向外翻转，负值=向内翻转

# 3. 右臂关节（target_joint.type=JOINT_RIGHT_ARM）
#   - pitch（抬起）：
#     范围：[-0.525, 2.66] rad （对应-30°~150°）
#     方向：负值=向下转动，正值=向上转动
#   - yaw（翻转）：
#     场景1：pitch < 2.4rad 时，范围[0.0, 0.525] rad（0°~30°）
#     场景2：pitch ≥ 2.4rad 时，范围[-0.26, 0.525] rad（-15°~30°）
#     方向：正值=向外翻转，负值=向内翻转