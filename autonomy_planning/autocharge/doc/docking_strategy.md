# 自动回充策略

自动回充分为两种使用方式：

1. **Odom + IR**：无 predock 位姿，仅用里程计与红外完成搜索、切入与接触。
2. **Odom + IR + Predock Pose**：先用 predock 位姿到达桩前并建立桩位先验，再进入红外对接；**δ 引导公式仅在此模式下使用**。

最终成功条件只有 `/dock/charge_connected == true`。

## 红外定义

```text
far_left  = 0x01
left      = 0x02
center    = 0x04
right     = 0x08
far_right = 0x10
```

侧别与有无信号一律用 **位与** 判断，不要求 token 精确等于单 bit：

```text
left_mask  = 0x01 | 0x02 | 0x04   # far_left + left + center
right_mask = 0x10 | 0x08 | 0x04   # far_right + right + center

# 例：右接收器看到组合束
(0x00, 0x10|0x08|0x04) = (0x00, 0x1C)
  → (right_token & right_mask) != 0  → 右侧有信号
  → (right_token & 0x04) != 0        → 含中心束

# 判边：指定接收器整 token 从有任意 bit → 连续 N 帧 0x00
# （不是 left/right emitter mask；mask 仍用于侧别/有无信号）
```

侧别按发射器 bit 判断。例如 `(0x10, 0x00)` → `FAR_RIGHT` → `BIAS_RIGHT_FAR`。

状态：

```text
SEARCH
  -> BIAS_LEFT_FAR / BIAS_RIGHT_FAR
  -> BIAS_LEFT / BIAS_RIGHT
  -> CENTER
  -> SUCCESS
```

---

# 策略一：Odom + IR（当前实现）

## 输入与原则

- **Odom**：机器人位姿、前进距离。
- **IR**：左右接收器五束 token；有无信号用位与 mask（左 `0x07` / 右 `0x1C`）。
- **名义桩位**：仅作粗参考，**不驱动 δ 引导线，不用红外红弧**。
- **BIAS**：按区域做 Z 字扫描——旋转到指定**接收器**「有任意 bit → 连续 N 帧 `0x00`」，再前进。
  判边看的是该接收器整 token，不是 left/right emitter mask（mask 仍用于侧别/有无信号）。
- **LEFT / LEFT_FAR**：先逆时针盯 right；**RIGHT / RIGHT_FAR**：先顺时针盯 left。
- **FAR 与 INNER 的差别**：FAR 首次判边 5 帧、前进 10cm；INNER 为 3 帧、5cm。出现内侧 `0x02/0x08` 时 FAR→INNER。
- **未见 0x04**：扫描累计满一圈后按当前 IR 重新判区，禁止未扫就盲前进。
- **FAR→INNER**：`(bits & 0x02/0x08)!=0` 才升级；不因远束闪烁降回 FAR。
- **丢束不盲冲**：BIAS 丢 IR 过久回 `SEARCH`；CENTER 丢 `0x04` 时 `vx=0`。

无 IR 时先 `SEARCH` 原地旋转；有名义桩位时朝桩方位转，`dist` 持续变差则翻转方向。

---

## 1. 在中间区域（CENTER）

### 典型观测

- 强中心：`(0x04, 0x04)`、稳定中心配对
- 弱中心：单侧或组合含 `0x04`

### 进入条件

1. 强中心确认 → 进入 `CENTER`
2. 弱中心且几何门控通过 → 进入 `CENTER`
3. 弱中心未过门控 → 按侧别进 `BIAS_*`，不空转锁中心

### 流程

```text
看见 0x04
  -> 左右 Z 字形微调（前进 v 很小，w 微调），始终让机器人看见 0x04，保持速度恒定前进
  -> 等待 charge_connected
  -> SUCCESS
```

### 控制要点

- 有 `0x04` 才前进。
- 禁止丢束后正 `vx` 盲冲。
- Z 字形左右微调。

---

## 2. 在 left 区域（内侧左）

### 典型观测

- `0x02`（left）
- 左侧重叠 / 左主导组合

### 进入条件

`SEARCH` 或其它状态确认左侧内区 → `BIAS_LEFT`

### 流程

```text
SEARCH / 其它
  -> 确认 LEFT
  -> BIAS_LEFT
  -> 看见 0x04 且确认 → CENTER，停止旋转
  -> 逆时针旋转至机器人的 right 红外连续 3 帧收到 00
     （须先见过 right 有信号再变 00；若旋转中看见 0x04 → CENTER）
     停止 → 前进 5cm
  -> 顺时针旋转至机器人的 left 红外连续 3 帧收到 00，停止
  -> 逆时针旋转至 right 连续 3 帧 00，停止 → 前进 5cm
  -> 若始终未见 0x04 且已旋转满一圈 → 按当前 IR 重新判区
  -> 循环直至 charge_connected → SUCCESS
```

---

## 3. 在 right 区域（内侧右）

### 典型观测

- `0x08`（right）
- 右侧重叠 / 右主导组合

### 进入条件

确认右侧内区 → `BIAS_RIGHT`

### 流程

```text
SEARCH / 其它
  -> 确认 RIGHT
  -> BIAS_RIGHT
  -> 看见 0x04 且确认 → CENTER，停止旋转
  -> 顺时针旋转至机器人的 left 红外连续 3 帧收到 00
     （须先见过 left 有信号再变 00；若旋转中看见 0x04 → CENTER）
     停止 → 前进 5cm
  -> 逆时针旋转至机器人的 right 红外连续 3 帧收到 00，停止
  -> 顺时针旋转至 left 连续 3 帧 00，停止 → 前进 5cm
  -> 若始终未见 0x04 且已旋转满一圈 → 按当前 IR 重新判区
  -> 循环直至 charge_connected → SUCCESS
```

---

## 4. 在 left_far 区域（远左）

### 典型观测

- `0x01`（far_left）
- FAR-only，无 inner / center bit

### 进入条件

确认 `FAR_LEFT` → `BIAS_LEFT_FAR`

### 流程

```text
SEARCH
  -> 确认 FAR_LEFT
  -> BIAS_LEFT_FAR
  -> 看见 0x04 且确认 → CENTER，停止旋转
  -> 逆时针旋转至 right 红外连续 5 帧收到 00，停止 → 前进 10cm
  -> 顺时针旋转至 left 红外连续 3 帧收到 00，停止
  -> 逆时针旋转至 right 连续 3 帧 00，停止 → 前进 10cm
  -> 若始终未见 0x04 且已旋转满一圈 → 按当前 IR 重新判区
  -> 循环；若出现内侧 LEFT → 升为 BIAS_LEFT
  -> charge_connected → SUCCESS
```

---

## 5. 在 right_far 区域（远右）

### 典型观测

- `0x10`（far_right）
- FAR-only，无 inner / center bit

### 进入条件

确认 `FAR_RIGHT` → `BIAS_RIGHT_FAR`

### 流程

```text
SEARCH
  -> 确认 FAR_RIGHT
  -> BIAS_RIGHT_FAR
  -> 看见 0x04 且确认 → CENTER，停止旋转
  -> 顺时针旋转至 left 红外连续 5 帧收到 00，停止 → 前进 10cm
  -> 逆时针旋转至 right 红外连续 3 帧收到 00，停止
  -> 顺时针旋转至 left 连续 3 帧 00，停止 → 前进 10cm
  -> 若始终未见 0x04 且已旋转满一圈 → 按当前 IR 重新判区
  -> 循环；若出现内侧 RIGHT → 升为 BIAS_RIGHT
  -> charge_connected → SUCCESS
```

---

# 策略二：Odom + IR + Predock Pose（摘要，未启用）

1. Predock 导航到桩前，并反推名义桩位。
2. 进入与策略一相同的区域状态机。
3. **仅在此模式下**启用 δ 引导：

```text
δ0 = β·(d + D)² + α
δ1 = δ0 · s_IR · s_lat
R  = clip(|v| / |w|)
δ  = (1 - λ)·δ1 + λ·R
x_target = x_current - δ
y_target = 0
```

4. δ 引导只允许与切入同向微调，不能改变 BIAS 转向。
5. 成功条件仍是 `charge_connected`。
