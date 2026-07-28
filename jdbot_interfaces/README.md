# jdbot_interfaces

JDBot 机器人 ROS2 接口定义包，包含所有 Topic（msg）、Service（srv）、Action 的完整定义。

---

## 目录

- [Quick Reference — ROS2 命令行速查](#quick-reference--ros2-命令行速查)
- [1. Topic (Message) 接口](#1-topic-message-接口)
  - [1.1 机器人状态](#11-机器人状态)
  - [1.2 头部/云台状态](#12-头部云台状态)
  - [1.3 手臂状态](#13-手臂状态)
  - [1.4 上半身状态](#14-上半身状态)
  - [1.5 关节控制](#15-关节控制)
  - [1.6 电池信息](#16-电池信息)
  - [1.7 底盘运动](#17-底盘运动)
  - [1.8 RL 观测与控制](#18-rl-观测与控制)
  - [1.9 RFID 找物](#19-rfid-找物)
  - [1.10 人体 Mask](#110-人体-mask)
  - [1.11 充电桩红外](#111-充电桩红外)
  - [1.12 拓扑导航](#112-拓扑导航)
  - [1.13 建图状态](#113-建图状态)
  - [1.14 SLAM 相关](#114-slam-相关)
  - [1.15 Terra 导航](#115-terra-导航)
  - [1.16 Embodied 具身控制](#116-embodied-具身控制)
  - [1.17 RL Debug](#117-rl-debug)
  - [1.18 机器人事件](#118-机器人事件)
  - [1.19 资源调度](#119-资源调度)
- [2. Service 接口](#2-service-接口)
  - [2.1 底盘服务](#21-底盘服务)
  - [2.2 头部/云台服务](#22-头部云台服务)
  - [2.3 手臂服务](#23-手臂服务)
  - [2.4 碰撞条服务](#24-碰撞条服务)
  - [2.5 语音服务](#25-语音服务)
  - [2.6 人脸服务](#26-人脸服务)
  - [2.7 RFID 找物服务](#27-rfid-找物服务)
  - [2.8 推流服务](#28-推流服务)
  - [2.9 MCU 版本](#29-mcu-版本)
  - [2.10 建图/定位服务 (mapping)](#210-建图定位服务-mapping)
  - [2.11 Terra 导航服务](#211-terra-导航服务)
  - [2.12 JDR_SLAM 服务](#212-jdr_slam-服务)
  - [2.13 快递柜/配送服务](#213-快递柜配送服务)
  - [2.14 拓扑路径规划](#214-拓扑路径规划)
  - [2.15 资源调度服务](#215-资源调度服务)
  - [2.16 Embodied 具身臂控制服务](#216-embodied-具身臂控制服务)
- [3. Action 接口](#3-action-接口)
  - [3.1 头部位置控制](#31-头部位置控制-setheadpos)
  - [3.2 RGBD 位置控制](#32-rgbd-位置控制-setrgbdpos)
  - [3.3 左臂位置控制 (2DOF)](#33-左臂位置控制-2dof-setarmleftpos)
  - [3.4 右臂位置控制 (2DOF)](#34-右臂位置控制-2dof-setarmrightpos)
  - [3.5 左臂位置控制 (4DOF)](#35-左臂位置控制-4dof-setarmleftposii)
  - [3.6 右臂位置控制 (4DOF)](#36-右臂位置控制-4dof-setarmrightposii)
  - [3.7 左臂预设动作](#37-左臂预设动作-setarmleftaction)
  - [3.8 右臂预设动作](#38-右臂预设动作-setarmrightaction)
  - [3.9 双臂预设动作](#39-双臂预设动作-setarmaction)
  - [3.10 上半身预设动作](#310-上半身预设动作-setupperaction)
  - [3.11 左臂循环动作](#311-左臂循环动作-setarmleftcycle)
  - [3.12 右臂循环动作](#312-右臂循环动作-setarmrightcycle)
  - [3.13 底盘高度控制](#313-底盘高度控制-setchassisheight)
  - [3.14 腰部位置控制](#314-腰部位置控制-setwaistposition)
  - [3.15 充电对接](#315-充电对接-dockrobot)
  - [3.16 自主探索](#316-自主探索-exploration)
  - [3.17 跟随](#317-跟随-follow)
  - [3.18 人脸注册](#318-人脸注册-faceregister)
  - [3.19 地图生成](#319-地图生成-mapgenerate)
- [4. 使用示例](#4-使用示例)
  - [4.1 Python 示例](#41-python-示例)
  - [4.2 命令行示例](#42-命令行示例)

---

## Quick Reference — ROS2 命令行速查

> 复制即用。所有命令按功能分组，覆盖本包全部 Topic / Service / Action 接口。

### Topic 订阅

```bash
# ---- 核心状态 ----
ros2 topic echo /robot_states            jdbot_interfaces/msg/RobotState            # 机器人整体状态(急停/碰撞/故障码/底盘模式)
ros2 topic echo /bms_info                jdbot_interfaces/msg/BmsInfo               # 电池信息(电量/电压/温度/保护状态)
ros2 topic echo /head_states             jdbot_interfaces/msg/HeadState             # 头部云台状态(yaw/pitch角度与速度)
ros2 topic echo /upper_body_states       jdbot_interfaces/msg/UpperBodyState        # 上半身状态(云台+左右臂角度/状态/错误码)
ros2 topic echo /joint_control_state     jdbot_interfaces/msg/JointControlState     # 关节控制状态汇总(各关节当前角度+底盘高度+腰部)
ros2 topic echo /joint_control_cmd       jdbot_interfaces/msg/JointControlCmd       # 关节控制指令(头部/左臂/右臂/RL控制)
ros2 topic echo /chassis_height          jdbot_interfaces/msg/SkateCmd              # 底盘高度反馈

# ---- 底盘 & 运动 ----
ros2 topic echo /skate_cmd               jdbot_interfaces/msg/SkateCmd              # 底盘运动指令(线速度/角速度/高度)
ros2 topic echo /cmd_vel                 geometry_msgs/msg/Twist                    # 底盘速度指令(标准ROS格式)
ros2 topic echo rl/observation           jdbot_interfaces/msg/RLObservation         # RL观测数据(关节速度/质心速度) 注意:无前导/

# ---- 感知 ----
ros2 topic echo /rgbd_pos_cmd            jdbot_interfaces/msg/RgbdPosCmd            # RGBD相机俯仰角指令
ros2 topic echo /ir/dock                 jdbot_interfaces/msg/DockIR                # 充电桩红外传感器数据

# ---- Embodied 具身 ----
ros2 topic echo /embodied/arm_states     jdbot_interfaces/msg/embodied/ArmStates    # 7DOF臂状态(关节位置/速度/力矩/TCP位姿)
ros2 topic echo /embodied/move_j         jdbot_interfaces/msg/embodied/MoveJ        # 关节空间运动指令(MoveJ)
ros2 topic echo /embodied/move_l         jdbot_interfaces/msg/embodied/MoveL        # 笛卡尔空间运动指令(MoveL)
ros2 topic echo /embodied/force_sensor_left   jdbot_interfaces/msg/embodied/ForceSensor   # 左臂力传感器(fx/fy/fz/mx/my/mz)
ros2 topic echo /embodied/force_sensor_right  jdbot_interfaces/msg/embodied/ForceSensor   # 右臂力传感器
ros2 topic echo /embodied/picker_cmd     jdbot_interfaces/msg/embodied/PickerCmd    # 夹爪控制指令(位置/速度/力/加速度)
ros2 topic echo /embodied/picker_status_left   jdbot_interfaces/msg/embodied/PickerStatus  # 左手夹爪状态
ros2 topic echo /embodied/picker_status_right  jdbot_interfaces/msg/embodied/PickerStatus  # 右手夹爪状态
ros2 topic echo /embodied/add_cartesian_waypoint jdbot_interfaces/msg/embodied/AddCartesianWaypoint  # 添加笛卡尔路点
ros2 topic echo /embodied/add_joint_waypoint     jdbot_interfaces/msg/embodied/AddJointWaypoint      # 添加关节路点
ros2 topic echo /embodied/debug          jdbot_interfaces/msg/embodied/Debug        # 具身调试信息(指令/反馈的位置/速度/力矩)

# ---- 事件 ----
ros2 topic echo /event/robot_event       jdbot_interfaces/msg/robot_event/RobotEvent  # 机器人事件(跌倒/关机/碰撞/挥手/手势等)
ros2 topic echo /power_off               std_msgs/msg/Empty                           # 关机信号
```

### Topic 发布

```bash
# 关节控制 — 头部向左转 0.5rad (type:3=头部, yaw正值=逆时针)
ros2 topic pub --once /joint_control_cmd jdbot_interfaces/msg/JointControlCmd \
  "{target_joint: {type: 3}, pitch: 0.0, yaw: 0.5}"

# 关节控制 — 左臂抬起 1.0rad (type:4=左臂, pitch正值=向上)
ros2 topic pub --once /joint_control_cmd jdbot_interfaces/msg/JointControlCmd \
  "{target_joint: {type: 4}, pitch: 1.0, yaw: 0.0}"

# 关节控制 — 右臂抬起 1.0rad (type:5=右臂)
ros2 topic pub --once /joint_control_cmd jdbot_interfaces/msg/JointControlCmd \
  "{target_joint: {type: 5}, pitch: 1.0, yaw: 0.0}"

# 底盘运动 — 前进0.3m/s
ros2 topic pub --once /skate_cmd jdbot_interfaces/msg/SkateCmd \
  "{xline_vel: 0.3, yline_vel: 0.0, wline_vel: 0.0, height: 0.0}"

# 具身臂 — MoveJ 关节空间运动 (arm_side:4=左臂, 速度比例0.3)
ros2 topic pub --once /embodied/move_j jdbot_interfaces/msg/embodied/MoveJ \
  "{arm_side: 4, motion_type: 1, q: [0,0,0,0,0,0,0], x: [0,0,0,0,0,0], qx_flag: 0, j_vscale: 0.3, j_accscale: 0.3}"

# 夹爪控制 — 左手半开 (hand_side:0x12=左手, posi_cmd:0~255对应0~120mm)
ros2 topic pub --once /embodied/picker_cmd jdbot_interfaces/msg/embodied/PickerCmd \
  "{hand_side: 18, posi_cmd: 128, vel_cmd: 100, force_cmd: 50, acc_cmd: 100, dec_cmd: 100}"
```

### Service 调用

```bash
# ---- 底盘 ----
ros2 service call /chassis/set_mode          jdbot_interfaces/srv/SetChassisMode       "{mode: 1}"            # 设置底盘模式 (1=平衡站立, 2=静态蹲坐)
ros2 service call /chassis/get_mode          jdbot_interfaces/srv/GetChassisMode       "{request_id: 'q'}"    # 获取底盘模式 (1=平衡, 2=静态, 3=折叠)
ros2 service call /chassis/get_height        jdbot_interfaces/srv/GetChassisHeight     "{request_id: 'q'}"    # 获取底盘高度 (相对/绝对/百分比)

# ---- 头部 / 云台 ----
ros2 service call /head/set_zero             jdbot_interfaces/srv/SetHeadZero          "{request_id: 'q'}"    # 头部回零 (yaw+pitch归零)
ros2 service call /head/set_yaw_zero         jdbot_interfaces/srv/SetHeadYawZero       "{request_id: 'q'}"    # 仅Yaw轴回零
ros2 service call /upper/set_max_vel         jdbot_interfaces/srv/SetUpperMaxVel       "{target_id: 3, max_velocity: 1.5, request_id: 'q'}"  # 设置上半身最大速度 (target_id: 3=头部, 4=左手, 5=右手; 范围[0,3]rad/s)

# ---- 手臂使能 ----
ros2 service call /arm/left/set_mode         jdbot_interfaces/srv/SetArmLeftMode       "{request_id: 'q', mode: 2}"   # 左臂使能 (1=失能, 2=使能)
ros2 service call /arm/right/set_mode        jdbot_interfaces/srv/SetArmRightMode      "{request_id: 'q', mode: 2}"   # 右臂使能

# ---- 碰撞条 ----
ros2 service call /chassis/set_bumper_mode       jdbot_interfaces/srv/SetBumperMode        "{mode: 1, request_id: 'q'}"                           # 开关碰撞条 (0=关闭, 1=打开)
ros2 service call /chassis/set_bumper_threshold  jdbot_interfaces/srv/SetBumperThreshold   "{bumper_threshold: [100,100,100,100,100], request_id: 'q'}"  # 设置碰撞阈值 (5个传感器)
ros2 service call /chassis/get_bumper_threshold  jdbot_interfaces/srv/GetBumperThreshold   "{request_id: 'q'}"                                    # 获取碰撞阈值

# ---- MCU 版本 ----
ros2 service call /robot/get_mcu_version     jdbot_interfaces/srv/GetMcuVersion        "{request_id: 'q'}"    # 获取MCU固件版本 (底盘+上身)

# ---- 建图 / 定位 (mapping) ----
ros2 service call /mapping/start_mapping       jdbot_interfaces/srv/mapping/StartMapping       "{request_id: 'q', mode: {mode: 1}}"   # 开始建图 (mode: 0=默认, 1=不存在时建, 2=重建, 3=增量)
ros2 service call /mapping/end_mapping         jdbot_interfaces/srv/mapping/EndMapping         "{request_id: 'q'}"                    # 结束建图
ros2 service call /mapping/start_localization  jdbot_interfaces/srv/mapping/StartLocalization  "{request_id: 'q'}"                    # 开始定位
ros2 service call /mapping/end_localization    jdbot_interfaces/srv/mapping/EndLocalization    "{request_id: 'q'}"                    # 结束定位
ros2 service call /mapping/get_mapping_status  jdbot_interfaces/srv/mapping/GetMappingStatus   "{request_id: 'q'}"                    # 查询建图状态与进度

# ---- Terra 导航 ----
ros2 service call /terra/start_exploration     jdbot_interfaces/srv/terra/StartExploration     "{node_id: '', node_name: 'floor1', mode: {mode: 1}}"   # 开始自主探索建图
ros2 service call /terra/end_exploration       jdbot_interfaces/srv/terra/EndExploration       "{}"                    # 结束探索
ros2 service call /terra/get_exploration_status jdbot_interfaces/srv/terra/GetExplorationStatus "{}"                    # 查询探索状态
ros2 service call /terra/start_node_mapping    jdbot_interfaces/srv/terra/StartNodeMapping     "{node_name: 'floor1', build_mode: {mode: 1}}"  # 开始节点建图
ros2 service call /terra/finish_mapping        jdbot_interfaces/srv/terra/FinishMapping        "{node_name: 'floor1'}"                         # 完成建图
ros2 service call /terra/load_map              jdbot_interfaces/srv/terra/LoadMap              "{node_name: 'floor1'}"                         # 加载地图
ros2 service call /terra/switch_node           jdbot_interfaces/srv/terra/SwitchNode           "{node_name: 'floor1', prior_poi_name: ''}"     # 切换节点(楼层)
ros2 service call /terra/list_maps             jdbot_interfaces/srv/terra/ListMaps             "{}"                    # 列出所有地图
ros2 service call /terra/get_map_info          jdbot_interfaces/srv/terra/GetMapInfo           "{map_name: 'floor1'}"  # 获取地图YAML信息
ros2 service call /terra/get_topology          jdbot_interfaces/srv/terra/GetTopology          "{request_id: 'q'}"     # 获取拓扑结构(节点+边)
ros2 service call /terra/get_slam_state        jdbot_interfaces/srv/terra/GetSLAMState         "{}"                    # 获取SLAM状态 (0=未就绪, 2=建图中, 3=定位中)
ros2 service call /terra/get_init_pose         jdbot_interfaces/srv/terra/GetInitPose          "{type: 1}"             # 获取初始位姿 (1=POSE, 2=检索, 3=TopK检索)
ros2 service call /terra/set_pose              jdbot_interfaces/srv/terra/SetPose              "{poi_name: 'entrance'}"                        # 设置位姿 (优先poi_name, 为空时用pose)
ros2 service call /terra/create_poi            jdbot_interfaces/srv/terra/CreatePoi            "{node_name: 'floor1', poi_name: 'desk_a', poi_type: 'waypoint', metadata: ''}"  # 创建兴趣点
ros2 service call /terra/delete_poi            jdbot_interfaces/srv/terra/DeletePoi            "{node_name: 'floor1', poi_name: 'desk_a'}"     # 删除兴趣点
ros2 service call /terra/query_poi             jdbot_interfaces/srv/terra/QueryPoi             "{node_name: 'floor1', poi_name: 'desk_a'}"     # 查询兴趣点
ros2 service call /terra/list_poi              jdbot_interfaces/srv/terra/ListPoi              "{map_name: 'floor1'}"                          # 列出所有兴趣点
ros2 service call /terra/delete_node           jdbot_interfaces/srv/terra/DeleteNode           "{node_name: 'floor1'}"                         # 删除拓扑节点
ros2 service call /terra/bind_topology_edge    jdbot_interfaces/srv/terra/BindTopologyEdge     "{from_poi_name: 'a', to_poi_name: 'b'}"        # 绑定拓扑边(电梯/楼梯)
ros2 service call /terra/active_relocation     jdbot_interfaces/srv/terra/ActiveRelocation     "{start: true, relocation_robot_type: ''}"      # 主动重定位

# ---- JDR_SLAM ----
ros2 service call /jdr_slam/map_database              jdbot_interfaces/srv/JDR_SLAM/MapDataBase              "{db_path: '/tmp/map.db', use_post_processing: false, thread_num: 4, incremental_mapping: false}"  # 地图数据库操作
ros2 service call /jdr_slam/set_pose_prior             jdbot_interfaces/srv/JDR_SLAM/SetPosePrior             "{}"             # 设置位姿先验
ros2 service call /jdr_slam/get_timestamped_pose       jdbot_interfaces/srv/JDR_SLAM/GetTimestampedPose       "{seconds: 0}"   # 查询时间戳对应的位姿
ros2 service call /jdr_slam/save_kf_and_gen_3d_map     jdbot_interfaces/srv/JDR_SLAM/SaveKeyFrameDataAndGenerate3DMap "{save_path: '/tmp/3dmap', thread_num: 4}"  # 保存关键帧并生成3D地图

# ---- 配送 ----
ros2 service call /local/set_top_lid     jdbot_interfaces/srv/package_deliever/SetTopLidLocal     "{request_id: 'q', mode: 1}"   # 顶盖控制 (1=打开, 2=关闭)
ros2 service call /local/set_tail_gate   jdbot_interfaces/srv/package_deliever/SetTailGateLocal   "{request_id: 'q', mode: 1}"   # 尾门控制 (1=打开, 2=关闭)

# ---- 拓扑路径规划 ----
ros2 service call /topo_route jdbot_interfaces/srv/TopoRoute \
  "{goal: {pose: {header: {frame_id: 'map'}}, sequence_number: 0, goal_id: 'target', name: 'floor1', type: 'normal'}}"  # 拓扑路径规划 (返回error_code: 0=成功)

# ---- 资源调度 ----
ros2 service call /scheduler/try_acquire_resource jdbot_interfaces/srv/scheduler/TryAcquireResource \
  "{source_request_id: 'q', resources: [{type: {type: 3}, count: 1}]}"  # 尝试获取资源 (type: 3=硬件头部, 4=硬件底盘)

# ---- Embodied 具身臂 (arm_side: 4=左臂, 5=右臂) ----
ros2 service call /embodied/get_cur_tcp_pose             jdbot_interfaces/srv/embodied/GetCurTcpPose              "{request_id: 'q', arm_side: 4}"   # 获取TCP位姿 [x,y,z,roll,pitch,yaw]
ros2 service call /embodied/get_cur_joints_pose          jdbot_interfaces/srv/embodied/GetCurJointsPose           "{request_id: 'q', arm_side: 4}"   # 获取7关节当前位置(rad)
ros2 service call /embodied/stop_motion                  jdbot_interfaces/srv/embodied/StopMotion                 "{request_id: 'q', arm_side: 4}"   # 停止运动
ros2 service call /embodied/pause_motion                 jdbot_interfaces/srv/embodied/PauseMotion                "{request_id: 'q', arm_side: 4}"   # 暂停运动
ros2 service call /embodied/resume_motion                jdbot_interfaces/srv/embodied/ResumeMotion               "{request_id: 'q', arm_side: 4}"   # 恢复运动
ros2 service call /embodied/cancel_motion                jdbot_interfaces/srv/embodied/CancelMotion               "{request_id: 'q', arm_side: 4}"   # 取消运动
ros2 service call /embodied/clear_error_code             jdbot_interfaces/srv/embodied/ClearErrorCode             "{request_id: 'q', arm_side: 4}"   # 清除错误码
ros2 service call /embodied/set_motion_mode              jdbot_interfaces/srv/embodied/SetMotionMode              "{request_id: 'q', arm_side: 4, motion_mode: 0}"  # 设置运动模式
ros2 service call /embodied/get_motion_mode              jdbot_interfaces/srv/embodied/GetMotionMode              "{request_id: 'q', arm_side: 4}"   # 获取运动模式
ros2 service call /embodied/get_tcp_offset               jdbot_interfaces/srv/embodied/GetTcpOffset               "{request_id: 'q', arm_side: 4}"   # 获取TCP偏移 [x,y,z,rx,ry,rz]
```

### Action 调用

```bash
# ---- 头部转动 (yaw范围[-2.618,2.618]rad, pitch范围[-0.489,1.745]rad, speed范围[0,2]rad/s) ----
ros2 action send_goal /head/set_pos jdbot_interfaces/action/SetHeadPos \
  "{yaw_angle: 0.5, pitch_angle: -0.2, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- RGBD相机转动 ----
ros2 action send_goal /rgbd/set_pos jdbot_interfaces/action/SetRgbdPos \
  "{yaw_angle: 0.0, pitch_angle: -0.3, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- 左臂2DOF (yaw范围[0,1.571], pitch范围[-1.571,1.571]) ----
ros2 action send_goal /arm/left/pos jdbot_interfaces/action/SetArmLeftPos \
  "{yaw_angle: 0.2, pitch_angle: 1.0, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- 右臂2DOF (yaw范围[0,1.571], pitch范围[-1.571,1.571]) ----
ros2 action send_goal /arm/right/pos jdbot_interfaces/action/SetArmRightPos \
  "{yaw_angle: 0.2, pitch_angle: 1.0, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- 左臂4DOF (肩部pitch/roll/yaw + 肘部) ----
ros2 action send_goal /arm/left/pos_ii jdbot_interfaces/action/SetArmLeftPosII \
  "{shoulder_pitch: 0.5, shoulder_roll: 0.0, shoulder_yaw: 0.0, elbow: 0.3, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- 右臂4DOF ----
ros2 action send_goal /arm/right/pos_ii jdbot_interfaces/action/SetArmRightPosII \
  "{shoulder_pitch: 0.5, shoulder_roll: 0.0, shoulder_yaw: 0.0, elbow: 0.3, max_speed: 1.0, request_id: 'q'}" --feedback

# ---- 左臂预设动作 (action_id: 预定义动作编号) ----
ros2 action send_goal /arm/left/set_action jdbot_interfaces/action/SetArmLeftAction \
  "{request_id: 'q', action_id: 1}" --feedback

# ---- 右臂预设动作 ----
ros2 action send_goal /arm/right/set_action jdbot_interfaces/action/SetArmRightAction \
  "{request_id: 'q', action_id: 1}" --feedback

# ---- 双臂预设动作 ----
ros2 action send_goal /arm/set_action jdbot_interfaces/action/SetArmAction \
  "{request_id: 'q', action_id: 1}" --feedback

# ---- 上半身预设动作 ----
ros2 action send_goal /upper/set_action jdbot_interfaces/action/SetUpperAction \
  "{request_id: 'q', action_id: 1}" --feedback

# ---- 左臂循环画圈 (pitch_base_rad:(-3.14,0), yaw_base_rad:(0,0.698), spread_level:[0,9]) ----
ros2 action send_goal /arm/left/set_cycle jdbot_interfaces/action/SetArmLeftCycle \
  "{pitch_base_rad: -1.0, yaw_base_rad: 0.3, spread_level: 5, cycles: 3, circle_dur_10ms: 200, request_id: 'q'}" --feedback

# ---- 右臂循环画圈 (yaw_base_rad:(-0.698,0), 注意与左臂方向相反) ----
ros2 action send_goal /arm/right/set_cycle jdbot_interfaces/action/SetArmRightCycle \
  "{pitch_base_rad: -1.0, yaw_base_rad: -0.3, spread_level: 5, cycles: 3, circle_dur_10ms: 200, request_id: 'q'}" --feedback

# ---- 底盘高度-绝对高度 (height_ab范围[0,10]cm, -255=不使用该模式) ----
ros2 action send_goal /chassis/set_height jdbot_interfaces/action/SetChassisHeight \
  "{height_re: -255.0, height_ab: 5.0, height_pct: -255.0, request_id: 'q'}" --feedback

# ---- 底盘高度-百分比 (height_pct范围[0,100]%) ----
ros2 action send_goal /chassis/set_height jdbot_interfaces/action/SetChassisHeight \
  "{height_re: -255.0, height_ab: -255.0, height_pct: 50.0, request_id: 'q'}" --feedback

# ---- 腰部转动 (position范围[-1.1,1.1]rad) ----
ros2 action send_goal /chassis/set_waist_position jdbot_interfaces/action/SetWaistPosition \
  "{position: 0.5, request_id: 'q'}" --feedback

# ---- 充电桩对接 ----
ros2 action send_goal /dock_robot jdbot_interfaces/action/DockRobot \
  "{dock_id: '', dock_type: '', navigate_to_staging_pose: true}" --feedback

# ---- 自主探索 (enable: true=开始, false=停止) ----
ros2 action send_goal /exploration jdbot_interfaces/action/Exploration \
  "{enable: true}" --feedback

# ---- 跟随目标 ----
ros2 action send_goal /follow jdbot_interfaces/action/Follow \
  "{poses: [], user_id: 'user_001'}" --feedback

# ---- 人脸注册 (command: JSON格式) ----
ros2 action send_goal /face_register jdbot_interfaces/action/FaceRegister \
  "{command: '{\"action\":\"register\",\"name\":\"test_user\"}'}" --feedback

# ---- 语义地图生成 ----
ros2 action send_goal /map_generate jdbot_interfaces/action/MapGenerate \
  "{input_path: '/tmp/raw_map'}" --feedback
```

> **提示**: 将 `arm_side: 4` 改为 `arm_side: 5` 即切换到右臂。`--feedback` 可省略。

---

## 1. Topic (Message) 接口

### 1.1 机器人状态

**`RobotState.msg`**

机器人全局状态，包含碰撞、故障码、底盘/云台/手臂状态等。

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳与坐标系 |
| `emergency` | `bool` | 急停状态 |
| `left_bumper` | `bool` | 左碰撞触发 |
| `right_bumper` | `bool` | 右碰撞触发 |
| `front_bumper` | `bool` | 前碰撞触发 |
| `leftback_bumper` | `bool` | 左后碰撞触发 |
| `rightback_bumper` | `bool` | 右后碰撞触发 |
| `fall_status` | `bool` | 机器跌倒 |
| `left_bumper_adc` | `uint8` | 左碰撞传感器 ADC 值 |
| `right_bumper_adc` | `uint8` | 右碰撞传感器 ADC 值 |
| `front_bumper_adc` | `uint8` | 前碰撞传感器 ADC 值 |
| `leftback_bumper_adc` | `uint8` | 左后碰撞传感器 ADC 值 |
| `rightback_bumper_adc` | `uint8` | 右后碰撞传感器 ADC 值 |
| `hip_err_l` | `uint8` | 左髋故障码（RS02 编码） |
| `hip_err_r` | `uint8` | 右髋故障码 |
| `knee_err_l` | `uint8` | 左膝故障码 |
| `knee_err_r` | `uint8` | 右膝故障码 |
| `wheel_err_l` | `uint8` | 左轮故障码（1505B 编码） |
| `wheel_err_r` | `uint8` | 右轮故障码 |
| `waist_status` | `uint8` | 腰部状态 |
| `waist_mot_err` | `uint8` | 腰部故障码 |
| `chassis_err` | `uint32` | 底盘错误码（位掩码） |
| `chassis_status` | `uint8` | 底盘状态 |
| `chassis_mode` | `uint8` | 底盘模式 |
| `camera_status` | `uint8` | 云台状态 |
| `camera_err` | `uint16` | 云台故障码（位掩码） |
| `arm_status_l` | `uint8` | 左臂状态 |
| `arm_err_l` | `uint16` | 左臂故障码 |
| `arm_status_r` | `uint8` | 右臂状态 |
| `arm_err_r` | `uint16` | 右臂故障码 |

**底盘模式枚举：**

| 常量 | 值 | 说明 |
|------|------|------|
| `MODE_BW_BASE_WHEEL_BALANCE` | 1 | 平衡模式（站立） |
| `MODE_BW_BASE_WHEEL_STATIC` | 2 | 静态模式（蹲坐） |
| `MODE_BW_BASE_WHEEL_FOLD` | 3 | 阻尼折叠模式 |

**设备状态枚举（camera_status / arm_status_l / arm_status_r）：**

| 常量 | 值 | 说明 |
|------|------|------|
| `STATUS_INIT` | 0 | 初始化中 |
| `STATUS_IDLE` | 1 | 空闲 |
| `STATUS_ERR` | 2 | 错误 |
| `STATUS_DISABLE` | 3 | 已禁用 |

**RS02 关节电机错误码（hip_err / knee_err）位掩码：**

| 位 | 常量 | 说明 |
|----|------|------|
| bit0 | `RS02_ERR_UNDER_VOLTAGE` (0x01) | 欠压 |
| bit1 | `RS02_ERR_OVER_CURRENT` (0x02) | 过流 |
| bit2 | `RS02_ERR_OVER_TEMPERATURE` (0x04) | 过温 |
| bit3 | `RS02_ERR_MAGNETIC_ENCODER` (0x08) | 磁编码器故障 |
| bit4 | `RS02_ERR_HALL_ENCODER` (0x10) | 霍尔编码器故障 |

**1505B 轮电机错误码（wheel_err_l / wheel_err_r）：**

| 常量 | 值 | 说明 |
|------|------|------|
| `WHEEL_ERR_NONE` | 0x00 | 无故障 |
| `WHEEL_ERR_UNDER_VOLTAGE_1` | 0x01 | 欠压1 (18V < Vbus < 20V) |
| `WHEEL_ERR_UNDER_VOLTAGE_2` | 0x02 | 欠压2 (Vbus < 18V) |
| `WHEEL_ERR_OVER_VOLTAGE` | 0x03 | 过压 (Vbus > 36V) |
| `WHEEL_ERR_OVER_CURRENT` | 0x0A | 过流 (Ibus > 35A) |
| `WHEEL_ERR_OVER_TEMP_1` | 0x20 | 过温1 (绕组 > 80°C) |
| `WHEEL_ERR_OVER_TEMP_2` | 0x1F | 过温2 (绕组 > 110°C) |
| `WHEEL_ERR_STALL` | 0x62 | 堵转 (电流 > 5A 且转速 = 0) |

---

### 1.2 头部/云台状态

**`HeadState.msg`**

| 字段 | 类型 | 说明 | 范围 |
|------|------|------|------|
| `yaw_rad` | `float32` | Yaw 角度 (rad) | ±150° |
| `pitch_rad` | `float32` | Pitch 角度 (rad) | ±56° |
| `current_yaw_speed` | `float32` | 当前 Yaw 角速度 (rad/s) | - |
| `current_pitch_speed` | `float32` | 当前 Pitch 角速度 (rad/s) | - |
| `state` | `uint16` | 云台状态码 | 见下表 |

**状态码枚举：**

| 常量 | 值 | 说明 |
|------|------|------|
| `ERROR_OK` | 0x0701 | 正常 |
| `HEAD_CTR_ERR_OK` | 0x0800 | 空闲停止 |
| `HEAD_CTR_ERR_FRAME` | 0x0801 | 数据校验失败 |
| `HEAD_CTR_ERR_MOVE` | 0x0802 | 运动中 |
| `HEAD_CTR_ERR_ODOM` | 0x0803 | 编码器数据出错 |
| `HEAD_CTR_ERR_ZERO` | 0x0805 | 校准零度中 |
| `HEAD_CTR_ERR_TIMEOUT` | 0x0806 | 电机堵转或机械错位，需手动清零 |

---

### 1.3 手臂状态

**`ArmLeftState.msg`** / **`ArmRightState.msg`**

与 `HeadState.msg` 结构相同，状态码枚举一致。

| 字段 | 类型 | 说明 |
|------|------|------|
| `yaw_rad` | `float32` | Yaw 角度 (rad) |
| `pitch_rad` | `float32` | Pitch 角度 (rad) |
| `current_yaw_speed` | `float32` | 当前 Yaw 角速度 (rad/s) |
| `current_pitch_speed` | `float32` | 当前 Pitch 角速度 (rad/s) |
| `state` | `uint16` | 状态码（枚举同 HeadState） |

---

### 1.4 上半身状态

**`UpperBodyState.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `camera_yaw_rad` | `float32` | 云台 Yaw (rad) |
| `camera_pitch_rad` | `float32` | 云台 Pitch (rad) |
| `camera_state` | `uint8` | 云台状态 |
| `camera_err_code` | `uint16` | 云台故障码 |
| `left_arm_roll_rad` | `float32` | 左臂 Roll (rad) |
| `left_arm_pitch_rad` | `float32` | 左臂 Pitch (rad) |
| `left_arm_state` | `uint8` | 左臂状态 |
| `left_arm_err_code` | `uint16` | 左臂故障码 |
| `right_arm_roll_rad` | `float32` | 右臂 Roll (rad) |
| `right_arm_pitch_rad` | `float32` | 右臂 Pitch (rad) |
| `right_arm_state` | `uint8` | 右臂状态 |
| `right_arm_err_code` | `uint16` | 右臂故障码 |
| `left_process_bar` | `uint8` | 左臂动作进度 |
| `right_process_bar` | `uint8` | 右臂动作进度 |

---

### 1.5 关节控制

**`JointControlCmd.msg`**

关节控制指令消息，支持头部、左臂、右臂及 RL 控制。

| 字段 | 类型 | 说明 |
|------|------|------|
| `target_joint` | `JointType` | 目标关节类型 |
| `pitch` | `float32` | 目标 Pitch 角度 (rad) |
| `yaw` | `float32` | 目标 Yaw 角度 (rad) |
| `head_control_type` | `RLControlType` | 头部 RL 控制类型 |
| `head_duration` | `Duration` | 头部动作持续时间 |
| `upper_control_type` | `RLControlType` | 上肢 RL 控制类型 |
| `upper_duration` | `Duration` | 上肢动作持续时间 |
| `lower_control_type` | `RLControlType` | 下肢 RL 控制类型 |
| `lower_duration` | `Duration` | 下肢动作持续时间 |

**JointType 枚举：**

| 常量 | 值 | 说明 |
|------|------|------|
| `JOINT_UNKNOWN` | 0 | 未知 |
| `JOINT_HEAD` | 3 | 头部 |
| `JOINT_LEFT_ARM` | 4 | 左臂 |
| `JOINT_RIGHT_ARM` | 5 | 右臂 |
| `JOINT_RL_CONTROL` | 100 | RL 控制模式 |

**各关节角度范围（v0.4.0 硬件）：**

| 关节 | 轴 | 范围 (rad) | 范围 (deg) | 方向 |
|------|------|------|------|------|
| 头部 | pitch | [-0.489, 1.745] | -28° ~ 100° | 负=下，正=上 |
| 头部 | yaw | [-2.618, 2.618] | -150° ~ 150° | 负=顺时针，正=逆时针 |
| 头部 | speed | [0.0, 2.0] | - | rad/s |
| 左臂 | pitch | [-1.571, 1.571] | -90° ~ 90° | 负=下，正=上 |
| 左臂 | yaw | [0.0, 1.571] | 0° ~ 90° | 正=外翻 |
| 右臂 | pitch | [-1.571, 1.571] | -90° ~ 90° | 负=下，正=上 |
| 右臂 | yaw | [0.0, 1.571] | 0° ~ 90° | 正=外翻 |

> **其他硬件版本范围参考** (源码 `constant_topics.hpp`)：
>
> | 版本 | 头部 yaw | 头部 pitch | 左臂 pitch | 右臂 yaw |
> |------|----------|------------|------------|----------|
> | v0.5.0 | [-1.571, 1.571] (-90°~90°) | [0, 1.571] (0°~90°) | [-3.142, 1.571] (-180°~90°) | [-1.571, 0] (-90°~0°) |
> | v0.8.0 | [-4.712, 1.571] (-270°~90°) | [-0.314, 0.698] (-18°~40°) | 同 v0.5.0 | 同 v0.5.0 |
> | small_w | [-1.571, 1.571] (-90°~90°) | [-0.35, 0.25] (-20°~14°) | - | - |

**`JointControlState.msg`**

各关节当前状态汇总。

| 字段 | 类型 | 说明 |
|------|------|------|
| `left_arm_yaw_rad` | `float32` | 左臂 Yaw (rad) |
| `left_arm_pitch_rad` | `float32` | 左臂 Pitch (rad) |
| `right_arm_yaw_rad` | `float32` | 右臂 Yaw (rad) |
| `right_arm_pitch_rad` | `float32` | 右臂 Pitch (rad) |
| `head_yaw_rad` | `float32` | 头部 Yaw (rad) |
| `head_pitch_rad` | `float32` | 头部 Pitch (rad) |
| `chassis_height_ab` | `float32` | 底盘绝对高度 |
| `waist_position` | `float32` | 腰部位置 |

**RLControlType 枚举：**

| 常量 | 值 | 说明 |
|------|------|------|
| `UNKNOWN_CONTROLTYPE` | 0 | 未知 |
| **头部动作** | | |
| `HEAD_SHAKE` | 100 | 左右摇头 |
| `HEAD_CENTER` | 101 | 头回正 |
| **上肢动作** | | |
| `UPPER_BODY_CENTER` | 200 | 上肢回正 |
| `BODY_RHYTHMIC_90_L` | 201 | 抬左手90度上下律动+腰 |
| `BODY_RHYTHMIC_90_R` | 202 | 抬右手90度上下律动+腰 |
| `ARM_RHYTHMIC_90_L` | 203 | 抬左手90度上下律动 |
| `ARM_RHYTHMIC_90_R` | 204 | 抬右手90度上下律动 |
| `ARM_RHYTHMIC_180_L` | 205 | 抬左手180度上下律动 |
| `ARM_RHYTHMIC_180_R` | 206 | 抬右手180度上下律动 |
| `LEFT_ARM_CIRCLE_90` | 207 | 左手90度画圈 |
| `RIGHT_ARM_CIRCLE_90` | 208 | 右手90度画圈 |
| `LEFT_ARM_CIRCLE_160` | 209 | 左手160度画圈 |
| `RIGHT_ARM_CIRCLE_160` | 210 | 右手160度画圈 |
| `ALTERNATE_RHYTHMIC` | 211 | 左右手错位律动 |
| `PENGUIN_ARM` | 212 | 企鹅手 |
| `ARMS_OPEN` | 213 | 双臂打开 |
| `ALTERNATE_SWING` | 214 | 交替挥臂 |
| `ARMS_RAISE_PUNCH` | 215 | 双臂举手挥拳 |
| `ARMS_CIRCLE_90` | 216 | 双手同时90度画圈 |
| `ARMS_CIRCLE_160` | 217 | 双手同时160度画圈 |
| **下肢动作** | | |
| `LOWER_BODY_CENTER` | 300 | 下肢回正 |
| `SPIN_IN_PLACE` | 301 | 原地转圈 |
| `RHYTHMIC_LEG_SHAKE` | 302 | 律动抖腿 |
| `MOONWALK` | 303 | 太空步 |
| `ROLL_SWING` | 304 | Roll 摆 |
| `BOW` | 305 | 弯腰 |

---

### 1.6 电池信息

**`BmsInfo.msg`**

| 字段 | 类型 | 说明 | 范围 |
|------|------|------|------|
| `level` | `uint8` | 电量百分比 | 0 ~ 100% |
| `pt_status` | `uint16` | 保护状态位（位掩码） | 见下表 |
| `power` | `float32` | 功率 (W)，正=放电，负=充电 | - |
| `voltage` | `float32` | 电压 (V) | - |
| `temperature` | `int8` | 温度 (C) | - |

**pt_status 保护位：**

| 位 | 常量 | 说明 |
|----|------|------|
| bit0 | `CELL_OVER_VOLTAGE` | 单体过压 |
| bit1 | `CELL_UNDER_VOLTAGE` | 单体欠压 |
| bit2 | `PACK_OVER_VOLTAGE` | 整组过压 |
| bit3 | `PACK_UNDER_VOLTAGE` | 整组欠压 |
| bit4 | `CHARGE_OVER_TEMP` | 充电过温 |
| bit5 | `CHARGE_UNDER_TEMP` | 充电低温 |
| bit6 | `DISCHARGE_OVER_TEMP` | 放电过温 |
| bit7 | `DISCHARGE_UNDER_TEMP` | 放电低温 |
| bit8 | `CHARGE_OVER_CURRENT` | 充电过流 |
| bit9 | `DISCHARGE_OVER_CURRENT` | 放电过流 |
| bit10 | `SHORT_CIRCUIT` | 短路 |
| bit11 | `FRONTEND_IC_ERROR` | 前端检测 IC 错误 |
| bit12 | `SOFTWARE_LOCK_MOS` | 软件锁定 MOS |

---

### 1.7 底盘运动

**`SkateCmd.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `xline_vel` | `float32` | X 方向线速度 |
| `yline_vel` | `float32` | Y 方向线速度 |
| `wline_vel` | `float32` | 旋转角速度 |
| `height` | `float32` | 高度 |

---

### 1.8 RL 观测与控制

**`RLObservation.msg`**

强化学习观测数据。

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 |
| `vel_hip_l` | `float64` | 左髋关节速度 (rad/s) |
| `vel_hip_r` | `float64` | 右髋关节速度 (rad/s) |
| `vel_knee_l` | `float64` | 左膝关节速度 (rad/s) |
| `vel_knee_r` | `float64` | 右膝关节速度 (rad/s) |
| `vel_wheel_l` | `float64` | 左轮速度 (rad/s) |
| `vel_wheel_r` | `float64` | 右轮速度 (rad/s) |
| `com_vel_linear_x` | `float64` | 质心线速度 (m/s) |
| `com_vel_yaw` | `float64` | 质心角速度 (rad/s) |
| `diff_vel_linear_x` | `float64` | 差分模型线速度 (m/s) |
| `diff_vel_yaw` | `float64` | 差分模型角速度 (rad/s) |
| `camera_yaw` | `float64` | 相机 Yaw (rad) |
| `camera_pitch` | `float64` | 相机 Pitch (rad) |
| `status_code` | `uint8` | 状态位 (0=OK) |

---

### 1.9 RFID 找物

**`FindItemInfo.msg`**

| 字段 | 类型 | 说明 | 范围 |
|------|------|------|------|
| `item_mac` | `string` | 标签 MAC 值 | - |
| `ant_id` | `uint32` | 天线 ID | 0 ~ 1 |
| `angle` | `float32` | 角度 (deg) | 0 ~ 180 |
| `rssi` | `float32` | 信号强度 (dBm) | -128 ~ 0 |
| `phase` | `float32` | 相位 | - |
| `distance` | `float32` | 天线与标签距离 (m) | - |
| `state` | `uint16` | E710 状态 | 见下表 |

| 常量 | 值 | 说明 |
|------|------|------|
| `RFID_OK` | 0 | 正常 |
| `RFID_ERR_ANT` | 1 | 天线连接异常 |
| `RFID_ERR_PORT` | 2 | 串口异常 |
| `RFID_ERR_CHECKSUM` | 4 | 串口帧校验错误 |

---

### 1.10 人体 Mask

**`PersonMask.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 |
| `width` | `int32` | 图像宽度 |
| `height` | `int32` | 图像高度 |
| `semantic_mask` | `uint8[]` | 语义分割 mask (0=人体, 1=背景) |
| `filtered_depth_image` | `uint16[]` | 过滤后深度图 (mm)，人体区域为 0 |

---

### 1.11 充电桩红外

**`DockIR.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 |
| `left` | `uint8` | 左侧红外值（低5位有效） |
| `right` | `uint8` | 右侧红外值（低5位有效） |
| `code` | `int32` | 状态码 |
| `message` | `string` | 消息 |

---

### 1.12 拓扑导航

**`TopoNode.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `pose` | `geometry_msgs/PoseStamped` | 节点位姿 |
| `sequence_number` | `uint32` | 序列号 |
| `goal_id` | `string` | 目标 ID |
| `name` | `string` | 楼层名 |
| `type` | `string` | 节点类型 |

**`TopoEdge.msg`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `sequence_number` | `uint32` | 序列号 |
| `start` | `TopoNode` | 起点 |
| `end` | `TopoNode` | 终点 |
| `type` | `uint8` | 移动类型 |

| 常量 | 值 | 说明 |
|------|------|------|
| `MOVETYPE_UNKNOWN` | 0 | 未知 |
| `MOVETYPE_ELEVATOR` | 1 | 电梯 |
| `MOVETYPE_STAIRCASE` | 2 | 楼梯 |
| `MOVETYPE_FLAT` | 3 | 平地 |

---

### 1.13 建图状态

**`mapping/MappingStatus.msg`**

| 常量 | 值 | 说明 |
|------|------|------|
| `STATUS_UNKNOWN` | 0 | 未知 |
| `STATUS_INIT` | 1 | 初始化 |
| `STATUS_BUSY` | 2 | 忙碌 |
| `STATUS_PROCESSING` | 3 | 处理中 |
| `STATUS_OK` | 4 | 完成 |
| `STATUS_ERROR` | 5 | 错误 |
| `STATUS_FINISH` | 6 | 结束 |

**`mapping/BuildMode.msg`**

| 常量 | 值 | 说明 |
|------|------|------|
| `DEFAULT_MODE` | 0 | 默认 |
| `BUILD_IF_NOT_EXIST` | 1 | 不存在时建图 |
| `REBUILD` | 2 | 重建 |
| `INCREMENTAL` | 3 | 增量建图 |

**`mapping/MappingProcessStage.msg`**

建图过程中的详细阶段，包含 INIT(100)、BUSY 系列(201-207)、PROCESSING 系列(301-302)、FINISH(400)、ERROR(500)、OK(1000) 等。

---

### 1.14 SLAM 相关

**`JDR_SLAM/SLAMState.msg`**

| 常量 | 值 | 说明 |
|------|------|------|
| `SYSTEM_NOT_READY` | 0 | 系统未就绪 |
| `IMU_INITIALIZING` | 1 | IMU 初始化中 |
| `MAPPING` | 2 | 建图中 |
| `LOCALIZING` | 3 | 定位中 |

**`JDR_SLAM/SLAMInfo.msg`** — 回环检测信息，含回环类型、大变化 ID、新旧地图变换矩阵。

**`JDR_SLAM/KeyFrame.msg`** — 关键帧数据，含位姿、地面/障碍点云、特征描述子。

**`JDR_SLAM/MapData.msg`** — 地图数据，含关键帧数组和链接关系（里程计/回环）。

---

### 1.15 Terra 导航

**`terra/TerraStatus.msg`**

| 常量 | 值 | 说明 |
|------|------|------|
| `SUCCESS` | 0 | 成功 |
| `ERROR_NOT_FOUND` | 1 | 未找到 |
| `ERROR_ALREADY_EXISTS` | 2 | 已存在 |
| `ERROR_LOCALIZATION_FAILED` | 3 | 定位失败 |
| `ERROR_INVALID_PARAM` | 4 | 参数无效 |
| `ERROR_ALREADY_LOCATED` | 5 | 已定位 |
| `ERROR_INTERNAL` | 255 | 内部错误 |

**`terra/TerraSLAMState.msg`** — NOT_READY(0) / MAPPING(2) / LOCALIZING(3)

**`terra/Topology.msg`** — 拓扑结构：nodes(TopologyNode[]) + edges(TopologyEdge[]) + current_node_name

**`terra/TopologyNode.msg`** — node_id, node_name, node_type, pois(POI[])

**`terra/TopologyEdge.msg`** — edge_id, edge_type(ELEVATOR=0/STAIRS=1), from_node_id, to_node_id

**`terra/POI.msg`** — poi_id, poi_name, poi_type, node_name, position(geometry_msgs/Pose)

---

### 1.16 Embodied 具身控制

**`embodied/ArmStates.msg`** — 7DOF 臂状态

| 字段 | 类型 | 说明 |
|------|------|------|
| `arm_side` | `uint8` | 左臂=0, 右臂=1 |
| `arm_status` | `uint8` | 运行状态 |
| `error_code` | `uint8` | 错误码 |
| `joint_pos` | `float32[7]` | 7 关节位置 (rad) |
| `joint_vel` | `float32[7]` | 7 关节速度 (rad/s) |
| `joint_torque` | `float32[7]` | 7 关节力矩 (N*m) |
| `tcp_pose` | `float32[6]` | TCP 位姿 [x,y,z,roll,pitch,yaw] |
| `tcp_twist` | `float32[6]` | TCP 速度 |

**`embodied/MoveJ.msg`** — 关节空间运动指令

| 字段 | 类型 | 说明 | 范围 |
|------|------|------|------|
| `arm_side` | `uint8` | 左臂=0x04, 右臂=0x05 | - |
| `motion_type` | `uint8` | 运动类型，1=MOVE_J | - |
| `q` | `float32[7]` | 7 关节目标位置 | - |
| `x` | `float32[6]` | 笛卡尔目标位置 | - |
| `qx_flag` | `uint8` | 有效目标位置标志 | - |
| `j_vscale` | `float32` | 速度比例 | [0.05, 1] |
| `j_accscale` | `float32` | 加速度比例 | [0.05, 1] |

**`embodied/MoveL.msg`** — 笛卡尔空间运动指令（结构同 MoveJ，使用 `l_vscale`/`l_accscale`）

**`embodied/ForceSensor.msg`** — 力传感器：fx/fy/fz (N), mx/my/mz (Nm)

**`embodied/PickerCmd.msg`** — 夹爪控制：posi_cmd(0-255/120mm), vel_cmd, force_cmd(0-255/30N), acc_cmd, dec_cmd

**`embodied/PickerStatus.msg`** — 夹爪状态：左手=0x12, 右手=0x13

**`embodied/EmHeadStates.msg`** — 具身头部状态：yaw/pitch 状态码、错误码、角度

**`embodied/AddJointWaypoint.msg`** — 关节路点：arm_side(左=0/右=1), q[7]

**`embodied/AddCartesianWaypoint.msg`** — 笛卡尔路点：arm_side(左=0/右=1), x[6]

**`embodied/Debug.msg`** — 调试信息：包含指令/反馈的关节位置、速度、力矩

---

### 1.17 RL Debug

**`rl_debug/RlTopic.msg`** — RL 调试总消息，聚合以下子消息：

| 子消息 | 说明 |
|------|------|
| `SystemStatus` | 错误码、状态、模式、事件、灯光、时间戳 |
| `RobotPose` | 里程计位姿 (x, y, theta)、速度 (vx, wz) |
| `JointGroup` | 6 关节组(left_hip/right_hip/left_knee/right_knee/left_wheel/right_wheel) + 腰部 |
| `JointState` | 单关节：position(rad), velocity(rad/s), executed_action, torque(Nm), torque_cmd(Nm) |
| `ImuData` | 陀螺仪 (rad/s) + 加速度计 (m/s^2) + 欧拉角 (rad) |
| `EkfEstimate` | EKF 质心位置/速度估计 |
| `ExtCmd` | 外部命令：vx/wz (输入+处理后) + height + 补偿量 |
| `Inference` | 网络推理：controller_name、obs、output_action、executed_action |

---

### 1.18 机器人事件

**`robot_event/RobotEvent.msg`**

| 事件常量 | 值 | 说明 |
|------|------|------|
| `EVENT_ROBOT_FALLDOWN` | 1001 | 机器人跌倒 |
| `EVENT_ROBOT_POWEROFF` | 1002 | 关机 |
| `EVENT_PLANNING_GLARE` | 6001 | 眩光 |
| `EVENT_PLANNING_LOW_LIGHT` | 6002 | 低光照 |
| `EVENT_PLANNING_BREAK_FREE` | 6003 | 挣脱 |
| `EVENT_PLANNING_STUCK_CONDITION` | 6004 | 卡死 |
| `EVENT_PLANNING_COLLISION` | 6005 | 碰撞 |
| `EVENT_PLANNING_EMERGENCY_STOP` | 6006 | 急停 |
| `EVENT_SLAM_LOST_POSITION` | 6008 | SLAM 丢失定位 |
| `EVENT_PLANNING_OBSTACLE_AVOIDING` | 6016 | 避障中 |
| `EVENT_AUTONOMOUS_SIT` | 8101 | 自主蹲坐 |
| `EVENT_ALGO_FALLDOWN` | 9001 | 算法检测跌倒 |
| `EVENT_ALGO_WATER` | 9002 | 检测到水 |
| `EVENT_ALGO_WAVEHAND` | 9003 | 检测到挥手 |
| `EVENT_ALGO_GESTURE` | 9004 | 检测到手势 |

事件消息体中包含对应的 payload：`FallDownEvent`、`PowerOffEvent`、`WaterEvent`、`WaveHandEvent`、`GestureEvent`。

---

### 1.19 资源调度

**`scheduler/Resource.msg`**

| 常量 | 值 | 说明 |
|------|------|------|
| `RESOURCE_TYPE_UNSPECIFIED` | 0 | 未指定 |
| `RESOURCE_TYPE_CPU` | 1 | CPU |
| `RESOURCE_TYPE_MEM` | 2 | 内存 |
| `RESOURCE_TYPE_HW_HEAD` | 3 | 硬件-头部 |
| `RESOURCE_TYPE_HW_CHASSIS` | 4 | 硬件-底盘 |

字段：`type`(uint8), `total`(int32), `exclusive`(bool)

---

## 2. Service 接口

### 2.1 底盘服务

**`SetChassisMode.srv`**

设置底盘工作模式。

```
# Request
uint8 mode          # 1=balance(站立), 2=static(蹲坐)
---
# Response
bool success
string message
```

**`GetChassisMode.srv`**

获取当前底盘模式。

```
# Request
string request_id
---
# Response
uint8 mode          # 1=balance, 2=static, 3=fold
bool success
string message
```

**`GetChassisHeight.srv`**

获取底盘高度。

```
# Request
string request_id
---
# Response
float32 cur_height_re   # 相对高度 (cm), [-5, 5]
float32 cur_height_ab   # 绝对高度 (cm), [0, 10]
float32 cur_height_pct  # 高度百分比, [0, 100]
bool success
string message
```

---

### 2.2 头部/云台服务

**`SetHeadZero.srv`** — 头部回零（yaw + pitch）

```
string request_id -> bool success, string message
```

**`SetHeadYawZero.srv`** — 仅 Yaw 轴回零

```
string request_id -> bool success, string message
```

**`SetHeadClearError.srv`** — 清除电机超时错误标志位

```
uint8 request (固定=0) -> bool success
```

> 注意：清除前请确认运动路径无卡位。

**`SetHeadMotorStop.srv`** — 停止所有云台电机供电

```
uint8 request (固定=1) -> bool success
```

**`SetHeadParam.srv`** — 设置硬件参数（出厂设置）

```
uint8 type  # 3=云台, 4=左手, 5=右手
-> bool success
```

**`GetHeadVersion.srv`** — 获取头部固件版本

```
(无请求参数) -> string version  # 格式 "Vx.x.x:HWx.x.x"
```

**`GetHeadReset.srv`** — 头部复位

```
(无请求参数) -> uint16 result  # 0=成功
```

**`SetUpperMaxVel.srv`** — 设置上半身最大速度

```
uint8 target_id      # 3=头部, 4=左手, 5=右手
float32 max_velocity  # [0, 3] rad/s (掉电丢失)
string request_id
-> bool success, string message
```

---

### 2.3 手臂服务

**`SetArmLeftMode.srv`** / **`SetArmRightMode.srv`**

手臂使能/失能。

```
string request_id
uint8 mode        # 1=disable(失能), 2=enable(使能)
---
bool success
string message
```

---

### 2.4 碰撞条服务

**`SetBumperMode.srv`**

```
uint8 mode          # 0=关闭碰撞条, 1=打开碰撞条
string request_id
---
bool success
string message
```

**`SetBumperThreshold.srv`**

```
uint16[5] bumper_threshold  # 5 个碰撞传感器阈值
string request_id
---
bool success
string message
```

**`GetBumperThreshold.srv`**

```
string request_id
---
uint16 front_threshold
uint16 leftback_threshold
uint16 right_threshold
uint16 rightback_threshold
uint16 left_threshold
bool success
string message
```

---

### 2.5 语音服务

**`SentAudio.srv`**

发送 TTS 语音播报。

```
string request       # 希望播报的文字
---
bool success
string message
```

---

### 2.6 人脸服务

**`FaceLogin.srv`**

```
string request_json   # JSON 格式请求
---
string response_json  # JSON 格式响应
```

---

### 2.7 RFID 找物服务

**`SetFindItemParam.srv`**

```
uint8 ant_id         # 天线 ID
bool switch_flag     # 开关
bool scan            # 扫描
bool selftest        # 自检
string mac           # 标签 MAC
---
string message
bool result
```

---

### 2.8 推流服务

**`SetPushStreamParam.srv`**

```
# Request
uint32 request_type
string stream_id
uint32 width, height, frame_rate, port
string ip_address, frame_type
uint32 rtp_ssrc
string camera_id
---
# Response (同结构 + status)
uint32 status
...
```

---

### 2.9 MCU 版本

**`GetMcuVersion.srv`**

```
string request_id
---
string chassis_version
string upper_version
bool success
string message
```

---

### 2.10 建图/定位服务 (mapping)

**`mapping/StartMapping.srv`**

```
string request_id
builtin_interfaces/Time timestamp
BuildMode mode
otel_interface/TraceCarrier trace
---
builtin_interfaces/Time timestamp
uint8 error_code
MappingStatus mapping_status
MappingProcessStage mapping_process_stage
float32 percent
```

**`mapping/EndMapping.srv`** — 结构同 StartMapping 响应

**`mapping/StartLocalization.srv`** / **`mapping/EndLocalization.srv`**

```
Request: request_id + timestamp + trace
---
Response: timestamp + error_code + MappingStatus
```

**`mapping/GetMappingStatus.srv`** — 获取建图状态（含进度百分比）

---

### 2.11 Terra 导航服务

**`terra/StartExploration.srv`** — 开始自主探索建图

```
# Request
string node_id       # 目标地图节点 UUID（REBUILD/INCREMENTAL 必填）
string node_name     # 节点展示名（可选）
BuildMode mode
---
# Response
TerraStatus status
string node_id       # 实际绑定的节点 UUID
string node_name
```

**`terra/EndExploration.srv`** — 结束探索

**`terra/GetExplorationStatus.srv`** — 查询探索/建图状态（含进度、节点信息）

**`terra/StartNodeMapping.srv`** — 开始节点建图

```
string node_name
BuildMode build_mode
---
TerraStatus status
```

**`terra/FinishMapping.srv`** — 完成建图

**`terra/StartNextMapping.srv`** — 开始下一段建图（跟随模式）

```
string node_name
string edge_type
---
TerraStatus status
string new_map_id
```

**`terra/LoadMap.srv`** — 加载地图

```
string node_name -> TerraStatus status
```

**`terra/SwitchNode.srv`** — 切换节点

```
string node_name, string prior_poi_name -> TerraStatus status
```

**`terra/ListMaps.srv`** — 列出所有地图

```
(空请求) -> string[] map_names
```

**`terra/GetMapInfo.srv`** — 获取地图信息

```
string map_name -> string yaml_data
```

**`terra/GetTopology.srv`** — 获取拓扑结构

```
string request_id -> TerraStatus status, Topology topology
```

**`terra/GetSLAMState.srv`** — 获取 SLAM 状态

```
(空请求) -> TerraSLAMState state, TerraStatus status
```

**`terra/GetInitPose.srv`** — 获取初始位姿

```
# Request
uint8 type       # 1=POSE, 2=RETRIEVAL, 3=RETRIEVAL_TOPK
geometry_msgs/Pose pose
---
# Response
geometry_msgs/Pose pose
int32[] topk_kf_ids
float32[] topk_scores
TerraStatus status
```

**`terra/SetPose.srv`** — 设置位姿

```
string poi_name              # 优先使用 poi_name
geometry_msgs/Pose pose      # poi_name 为空时使用 pose (base_footprint->map)
---
TerraStatus status
```

**`terra/CreatePoi.srv`** / **`terra/DeletePoi.srv`** / **`terra/QueryPoi.srv`** / **`terra/ListPoi.srv`** — POI 增删查

**`terra/DeleteNode.srv`** — 删除节点

**`terra/BindTopologyEdge.srv`** — 绑定拓扑边

```
string from_poi_name, string to_poi_name -> TerraStatus status
```

**`terra/ActiveRelocation.srv`** — 主动重定位

```
bool start, string relocation_robot_type -> TerraStatus status
```

---

### 2.12 JDR_SLAM 服务

**`JDR_SLAM/MapDataBase.srv`**

```
string db_path
bool use_post_processing
string post_processing_dir_path
uint8 thread_num
bool incremental_mapping
---
bool success
string message
```

**`JDR_SLAM/SetPosePrior.srv`**

```
geometry_msgs/Pose position -> bool success, string message
```

**`JDR_SLAM/GetTimestampedPose.srv`**

```
uint64 seconds -> bool success, string message, geometry_msgs/Pose position
```

**`JDR_SLAM/SaveKeyFrameDataAndGenerate3DMap.srv`**

```
string save_path, uint8 thread_num -> bool success
```

---

### 2.13 快递柜/配送服务

**`package-deliever/OpenCompartment.srv`** / **`CloseCompartment.srv`** / **`ConnectCompartment.srv`** / **`SlideOutPackage.srv`**

```
string compartment_id
---
uint16 error_code
string error_msg
```

**`package-deliever/SetTopLidLocal.srv`** — 顶盖控制

```
string request_id
uint8 mode          # 1=MODE_OPEN(打开), 2=MODE_CLOSE(关闭)
---
bool success
string message
```

**`package-deliever/SetTailGateLocal.srv`** — 尾门控制（结构同上）

---

### 2.14 拓扑路径规划

**`TopoRoute.srv`**

```
# Request
TopoNode goal
---
# Response
uint8 error_code    # 0=成功, 1=节点未找到, 2=POI无效, 3=规划失败, 4=内部错误, 5=定位错误
TopoEdge[] paths
string message
```

---

### 2.15 资源调度服务

**`scheduler/TryAcquireResource.srv`**

```
string source_request_id
TryAcquireResourceRequestResource[] resources
---
bool ok
string error_message
```

---

### 2.16 Embodied 具身臂控制服务

> 以下服务中 `arm_side` 参数：**左臂=0x04, 右臂=0x05**

**使能控制：**

| 服务 | 说明 |
|------|------|
| `embodied/ArmEnable.srv` | 使能/失能：enable_cmd (1=使能, 0=失能) |
| `embodied/GetArmEnableStatus.srv` | 查询使能状态 -> is_enabled |
| `embodied/GetArmStatus.srv` | 查询臂状态 -> arm_status, joint_positions[7], is_moving 等 |
| `embodied/GetErrorCode.srv` | 查询错误码 |
| `embodied/ClearErrorCode.srv` | 清除错误码 |

**运动控制：**

| 服务 | 说明 |
|------|------|
| `embodied/PauseMotion.srv` | 暂停运动 |
| `embodied/ResumeMotion.srv` | 恢复运动 |
| `embodied/StopMotion.srv` | 停止运动 |
| `embodied/CancelMotion.srv` | 取消运动 |

**控制模式：**

**`embodied/SetControlMode.srv`**

```
uint8 control_mode:
  0 = JOINT_POSITION        (默认状态)
  1 = JOINT_IMPEDANCE       (关节阻抗)
  2 = JOINT_ADMITTANCE      (关节导纳)
  3 = CARTESIAN_POSITION    (笛卡尔位置)
  4 = CARTESIAN_IMPEDANCE   (笛卡尔阻抗)
  5 = CARTESIAN_ADMITTANCE  (笛卡尔导纳)
  6 = CARTESIAN_HYBRID      (笛卡尔力位混合)
```

**`embodied/SetMotionMode.srv`** / **`GetMotionMode.srv`** — 运动模式

**状态查询：**

| 服务 | 返回 |
|------|------|
| `GetCurJointsPose.srv` | float32[7] x — 7 关节当前位置 (rad) |
| `GetCurrentJointPositionArray.srv` | float32[7] joint_positions |
| `GetCurrentJointVelocitiesArray.srv` | float32[7] joint_velocity |
| `GetCurrentJointTorquesArray.srv` | float32[7] torques |
| `GetCurTcpPose.srv` | float32[6] x — TCP 位姿 [x,y,z,roll,pitch,yaw] (m/rad) |
| `GetCurTcpTwist.srv` | float32[6] twist — TCP 速度 |
| `GetControlMode.srv` | uint8 control_mode |
| `GetTcpOffset.srv` | float32[6] offset [x,y,z,rx,ry,rz] (m/rad) |

**参数设置/获取：**

| 服务 | 说明 |
|------|------|
| `SetTcpOffset.srv` | 设置 TCP 偏移 [x,y,z,rx,ry,rz] |
| `Set/GetGlobalJointVelocityScale.srv` | 关节速度比例 |
| `Set/GetGlobalJointAccelerationScale.srv` | 关节加速度比例 |
| `Set/GetGlobalCartesianVelocityScale.srv` | 笛卡尔速度比例 |
| `Set/GetGlobalCartesianAccelerationScale.srv` | 笛卡尔加速度比例 |
| `Set/GetJointImpedanceStiffness.srv` | 关节阻抗刚度 (float32[7]) |
| `Set/GetJointImpedanceDamping.srv` | 关节阻抗阻尼 (float32[7]) |
| `Set/GetJointAdmittanceStiffness.srv` | 关节导纳刚度 (float32[7]) |
| `Set/GetJointAdmittanceDamping.srv` | 关节导纳阻尼 (float32[7]) |
| `Set/GetCartesianImpedanceStiffness.srv` | 笛卡尔阻抗刚度 (float32[6]) |
| `Set/GetCartesianImpedanceDamping.srv` | 笛卡尔阻抗阻尼 (float32[6]) |
| `Set/GetHandGripperInertial.srv` | 末端质心惯性参数 (mass, xyz, ixx/iyy/izz/ixy/ixz/iyz) |

---

## 3. Action 接口

### 3.1 头部位置控制 (SetHeadPos)

Action 名称：`/head/set_pos`

```
# Goal
float32 yaw_angle       # 目标 Yaw 角度 (rad)
float32 pitch_angle     # 目标 Pitch 角度 (rad)
float32 max_speed       # 最大速度 (rad/s)
string request_id
---
# Result
bool success            # true=成功, false=失败
string message
---
# Feedback
float32 current_yaw     # 当前 Yaw (rad)
float32 current_pitch   # 当前 Pitch (rad)
```

---

### 3.2 RGBD 位置控制 (SetRgbdPos)

Action 名称：`/rgbd/set_pos`

结构同 SetHeadPos。

---

### 3.3 左臂位置控制 2DOF (SetArmLeftPos)

Action 名称：`/arm/left/pos`

```
# Goal
float32 yaw_angle       # 目标 Yaw (rad)
float32 pitch_angle     # 目标 Pitch (rad)
float32 max_speed       # 最大速度 (rad/s)
string request_id
otel_interface/TraceCarrier trace
---
# Result
bool success
string message
---
# Feedback
float32 current_yaw
float32 current_pitch
```

---

### 3.4 右臂位置控制 2DOF (SetArmRightPos)

Action 名称：`/arm/right/pos`

结构同 SetArmLeftPos。

---

### 3.5 左臂位置控制 4DOF (SetArmLeftPosII)

```
# Goal
float32 shoulder_pitch    # 肩部 Pitch (rad)
float32 shoulder_roll     # 肩部 Roll (rad)
float32 shoulder_yaw      # 肩部 Yaw (rad)
float32 elbow             # 肘部角度 (rad)
float32 max_speed         # 最大速度 (rad/s)
string request_id
otel_interface/TraceCarrier trace
---
# Result
bool success
string message
---
# Feedback
float32 current_shoulder_pitch
float32 current_shoulder_roll
float32 current_shoulder_yaw
float32 current_elbow
```

---

### 3.6 右臂位置控制 4DOF (SetArmRightPosII)

结构同 SetArmLeftPosII。

---

### 3.7 左臂预设动作 (SetArmLeftAction)

```
# Goal
string request_id
uint16 action_id          # 预设动作 ID
---
# Result
bool success
string message
---
# Feedback
uint8 progress            # 进度百分比 0~100
```

---

### 3.8 右臂预设动作 (SetArmRightAction)

结构同 SetArmLeftAction。

---

### 3.9 双臂预设动作 (SetArmAction)

Action 名称：`/arm/set_action`

```
# Goal
string request_id
uint16 action_id
---
# Result
bool success
string message
---
# Feedback
uint8 progress
```

---

### 3.10 上半身预设动作 (SetUpperAction)

Action 名称：`/upper/set_action`

结构同 SetArmAction。

---

### 3.11 左臂循环动作 (SetArmLeftCycle)

Action 名称：`/arm/left/set_cycle`

```
# Goal
float32 pitch_base_rad    # 基础 Pitch (rad), 范围 (-3.14, 0)
float32 yaw_base_rad      # 基础 Yaw (rad), 范围 (0, 0.698)
uint8 spread_level         # 展开等级, 范围 [0, 9]
int8 cycles                # 循环次数 (方向: 内侧)
uint16 circle_dur_10ms     # 单圈时长, 单位 10ms
string request_id
---
# Result
bool success
string message
---
# Feedback
uint8 progress
```

---

### 3.12 右臂循环动作 (SetArmRightCycle)

Action 名称：`/arm/right/set_cycle`

```
# Goal
float32 pitch_base_rad    # 范围 (-3.14, 0)
float32 yaw_base_rad      # 范围 (-0.698, 0)  <-- 注意与左臂相反
uint8 spread_level         # [0, 9]
int8 cycles
uint16 circle_dur_10ms
string request_id
---
# Result / Feedback 同左臂
```

---

### 3.13 底盘高度控制 (SetChassisHeight)

Action 名称：`/chassis/set_height`

支持三种模式：相对高度、绝对高度、百分比。使用 `-255` 表示不使用该模式。

```
# Goal
float32 height_re       # 相对高度 (cm), [-5, 5], -255=不使用
float32 height_ab       # 绝对高度 (cm), [0, 10], -255=不使用
float32 height_pct      # 百分比, [0, 100], -255=不使用
string request_id
---
# Result
bool success
string message
---
# Feedback
float32 cur_height_re   # 当前相对高度 (cm)
float32 cur_height_ab   # 当前绝对高度 (cm)
float32 cur_height_pct  # 当前百分比 (%)
```

---

### 3.14 腰部位置控制 (SetWaistPosition)

Action 名称：`/chassis/set_waist_position`

```
# Goal
float32 position        # 目标角度 (rad), 范围 [-1.1, 1.1]
string request_id
---
# Result
bool success
string message
---
# Feedback
float32 cur_position    # 当前角度 (rad)
```

---

### 3.15 充电对接 (DockRobot)

```
# Goal
string dock_id
string dock_type
geometry_msgs/PoseStamped pose
bool navigate_to_staging_pose
---
# Result
bool success
uint16 error_code
string error_text
---
# Feedback
string state                          # 当前运行步骤
builtin_interfaces/Duration time_remaining  # 预估剩余时间
uint16 num_retries                    # 重试次数
```

**错误码：**

| 常量 | 值 | 说明 |
|------|------|------|
| `NONE` | 0 | 无错误 |
| `DOCK_NOT_FOUND` | 1 | 未检测到二维码 |
| `DOCK_NOT_VALID` | 2 | 二维码数据无法解析 |
| `INTERNAL_ERROR` | 3 | 机器人控制异常 |
| `ALIGNMENT_FAILED` | 4 | 容差检查不通过 |
| `CODE_TF_FAILED` | 102 | TF 变换失败 |
| `CODE_INFERENCE_FAILED` | 103 | 推理失败 |
| `CODE_NAV_FAILED` | 104 | 导航失败 |

---

### 3.16 自主探索 (Exploration)

```
# Goal
bool enable
---
# Result
bool success
---
# Feedback
uint8 state
```

| Feedback 常量 | 值 | 说明 |
|------|------|------|
| `FEEDBACK_STATE_NONE` | 0 | 无 |
| `FEEDBACK_STATE_INACTIVE` | 1 | 未激活 |
| `FEEDBACK_STATE_ACTIVE` | 2 | 已激活 |
| `FEEDBACK_STATE_INACTIVING` | 3 | 正在停止 |
| `FEEDBACK_STATE_ACTIVING` | 4 | 正在启动 |

---

### 3.17 跟随 (Follow)

```
# Goal
geometry_msgs/PoseStamped[] poses   # 目标位姿序列
string user_id                       # 跟随目标用户
---
# Result
bool success
---
# Feedback
uint8 state
```

| Feedback 常量 | 值 | 说明 |
|------|------|------|
| `FEEDBACK_STATE_NONE` | 0 | 无 |
| `FEEDBACK_STATE_RUNNING` | 1 | 运行中 |
| `FEEDBACK_STATE_CANCELED_ERROR` | 2 | 取消/错误 |
| `FEEDBACK_STATE_START_ERROR` | 3 | 启动错误 |
| `FEEDBACK_STATE_SYSTEM_ERROR` | 4 | 系统错误 |

---

### 3.18 人脸注册 (FaceRegister)

```
# Goal
string command        # JSON 格式命令（注册或删除）
---
# Result
bool success
string message
---
# Feedback
string status         # 当前操作状态
```

---

### 3.19 地图生成 (MapGenerate)

```
# Goal
string input_path     # 输入文件路径
---
# Result
string result_path    # 语义地图文件路径
---
# Feedback
string status         # 处理状态
float32 process_ratio # 处理进度 [0.0, 1.0]
```

---

## 4. 使用示例

### 4.1 Python 示例

#### 订阅机器人状态 Topic

```python
import rclpy
from rclpy.node import Node
from jdbot_interfaces.msg import RobotState

class RobotStateListener(Node):
    def __init__(self):
        super().__init__('robot_state_listener')
        self.sub = self.create_subscription(
            RobotState, '/robot_states', self.callback, 10)

    def callback(self, msg: RobotState):
        self.get_logger().info(
            f'emergency={msg.emergency}, '
            f'chassis_mode={msg.chassis_mode}, '
            f'fall_status={msg.fall_status}')
        if msg.chassis_err != 0:
            self.get_logger().warn(f'底盘错误码: 0x{msg.chassis_err:08X}')

def main():
    rclpy.init()
    rclpy.spin(RobotStateListener())

if __name__ == '__main__':
    main()
```

#### 发布关节控制指令

```python
import rclpy
from rclpy.node import Node
from jdbot_interfaces.msg import JointControlCmd, JointType

class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')
        self.pub = self.create_publisher(JointControlCmd, '/joint_control_cmd', 10)

    def look_left(self):
        msg = JointControlCmd()
        msg.target_joint.type = JointType.JOINT_HEAD
        msg.yaw = 1.0     # 1.0 rad, 逆时针转
        msg.pitch = 0.0    # 水平
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = HeadController()
    node.look_left()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

#### 调用 Service：设置底盘模式

```python
import rclpy
from rclpy.node import Node
from jdbot_interfaces.srv import SetChassisMode

class ChassisModeClient(Node):
    def __init__(self):
        super().__init__('chassis_mode_client')
        self.cli = self.create_client(SetChassisMode, '/chassis/set_mode')
        self.cli.wait_for_service()

    def set_balance(self):
        req = SetChassisMode.Request()
        req.mode = 1  # balance mode
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'success={result.success}, msg={result.message}')

def main():
    rclpy.init()
    node = ChassisModeClient()
    node.set_balance()

if __name__ == '__main__':
    main()
```

#### 调用 Action：控制头部位置

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from jdbot_interfaces.action import SetHeadPos

class HeadPosClient(Node):
    def __init__(self):
        super().__init__('head_pos_client')
        self.client = ActionClient(self, SetHeadPos, '/head/set_pos')
        self.client.wait_for_server()

    def send_goal(self, yaw: float, pitch: float, speed: float = 1.0):
        goal = SetHeadPos.Goal()
        goal.yaw_angle = yaw       # rad, [-2.618, 2.618]
        goal.pitch_angle = pitch   # rad, [-0.489, 1.745]
        goal.max_speed = speed     # rad/s
        goal.request_id = 'demo_001'

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'success={result.success}, msg={result.message}')

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'yaw={fb.current_yaw:.3f}, pitch={fb.current_pitch:.3f}')

def main():
    rclpy.init()
    node = HeadPosClient()
    node.send_goal(yaw=0.5, pitch=-0.3, speed=1.5)

if __name__ == '__main__':
    main()
```

#### 调用 Action：设置底盘高度

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from jdbot_interfaces.action import SetChassisHeight

class ChassisHeightClient(Node):
    def __init__(self):
        super().__init__('chassis_height_client')
        self.client = ActionClient(self, SetChassisHeight, '/chassis/set_height')
        self.client.wait_for_server()

    def set_height_absolute(self, height_cm: float):
        """设置绝对高度, 范围 [0, 10] cm"""
        goal = SetChassisHeight.Goal()
        goal.height_re = -255.0     # 不使用相对高度
        goal.height_ab = height_cm  # 使用绝对高度
        goal.height_pct = -255.0    # 不使用百分比
        goal.request_id = 'height_001'

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'success={result.success}')

def main():
    rclpy.init()
    node = ChassisHeightClient()
    node.set_height_absolute(5.0)  # 设置到 5cm

if __name__ == '__main__':
    main()
```

#### 调用 Embodied 具身臂服务

```python
import rclpy
from rclpy.node import Node
from jdbot_interfaces.srv import (
    ArmEnable, SetControlMode, GetCurJointsPose, GetCurTcpPose
)

ARM_LEFT = 0x04
ARM_RIGHT = 0x05

class EmbodiedArmClient(Node):
    def __init__(self):
        super().__init__('embodied_arm_client')
        self.enable_cli = self.create_client(ArmEnable, '/embodied/arm_enable')
        self.mode_cli = self.create_client(SetControlMode, '/embodied/set_control_mode')
        self.joints_cli = self.create_client(GetCurJointsPose, '/embodied/get_cur_joints_pose')
        self.tcp_cli = self.create_client(GetCurTcpPose, '/embodied/get_cur_tcp_pose')

    def enable_left_arm(self):
        req = ArmEnable.Request()
        req.request_id = 'en_001'
        req.arm_side = ARM_LEFT
        req.enable_cmd = 1  # enable
        future = self.enable_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'enable: {future.result().success}')

    def set_joint_impedance_mode(self):
        req = SetControlMode.Request()
        req.request_id = 'mode_001'
        req.arm_side = ARM_LEFT
        req.control_mode = 1  # JOINT_IMPEDANCE
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'set_mode: {future.result().success}')

    def get_left_arm_joints(self):
        req = GetCurJointsPose.Request()
        req.request_id = 'q_001'
        req.arm_side = ARM_LEFT
        future = self.joints_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'joints (rad): {list(result.x)}')

    def get_left_tcp_pose(self):
        req = GetCurTcpPose.Request()
        req.request_id = 'tcp_001'
        req.arm_side = ARM_LEFT
        future = self.tcp_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'TCP [x,y,z,r,p,y]: {list(result.x)}')

def main():
    rclpy.init()
    node = EmbodiedArmClient()
    node.enable_left_arm()
    node.set_joint_impedance_mode()
    node.get_left_arm_joints()
    node.get_left_tcp_pose()

if __name__ == '__main__':
    main()
```

---

> 完整命令行速查见文档顶部 [Quick Reference](#quick-reference--ros2-命令行速查) 章节。

---

## 附录：arm_side 参数对照表

在不同上下文中 `arm_side` 的取值有差异，使用时请注意：

| 上下文 | 左 | 右 |
|--------|------|------|
| Embodied srv (具身臂服务) | 0x04 | 0x05 |
| Embodied msg (ArmStates 等) | 0 | 1 |
| PickerCmd / PickerStatus (夹爪) | 0x12 (左手) | 0x13 (右手) |
| MoveJ / MoveL (运动指令) | 0x04 | 0x05 |

---

## 注意事项

1. **角度单位**: 所有角度参数均使用**弧度 (rad)**，注意不要与角度 (deg) 混淆
2. **-255 约定**: 在 SetChassisHeight 中，参数值为 `-255` 表示该字段不使用
3. **request_id**: 大部分接口需要传入 `request_id`，建议使用唯一标识便于追踪
4. **运动安全**: 控制关节前请确认机器人处于安全状态，避免碰撞或损坏
5. **模式切换**: 底盘模式切换需在对应姿态下进行（balance=站立, static=蹲坐）
6. **错误处理**: 关注返回的 `success` 字段和 `message` 描述，出错时检查错误码
7. **otel trace**: 部分接口含 `otel_interface/TraceCarrier trace` 字段，用于链路追踪
