# 自动回充文档

本目录给出当前 `autocharge` 的红外布局说明、最优回充策略，以及测试和调优建议。

推荐阅读顺序：

1. [`ir_layout_and_limits.md`](./ir_layout_and_limits.md)  
   先理解充电桩发射器、机器人接收器、协议编码和成功判定的物理限制。

2. [`docking_strategy.md`](./docking_strategy.md)  
   再理解在当前布局下，`SEARCH`、`BIAS_LEFT`、`BIAS_RIGHT`、`CENTER` 应如何协同工作。

3. [`testing_and_tuning.md`](./testing_and_tuning.md)  
   最后按照建议的测试点位和日志判据验证成功率。

## 一句话策略

最优运行方式是：

```text
predock / 导航入区
  -> SEARCH 优先抓中心（侧边整圈后再选）
  -> BIAS_LEFT 或 BIAS_RIGHT 闭环保信号收敛
  -> CENTER 见 0x04 才前进，丢束停转回找
  -> charge_connected
  -> SUCCESS
```

## 相关实现文件

- 状态机实现：`autocharge/docking/fsm.py`
- 控制节点：`autocharge/docking/node.py`
- 仿真与发射器模型：`autocharge/sim/env_sim.py`
- 配置：`config/sim.yaml`、`config/docking.yaml`

## 文档使用范围

本文档面向当前这套五路发射器布局与双接收器布局。如果后续充电桩物理安装位置、波束角度或编码规则改变，应优先更新：

1. `config/sim.yaml`
2. `config/docking.yaml`
3. `doc/ir_layout_and_limits.md`
4. `doc/docking_strategy.md`
