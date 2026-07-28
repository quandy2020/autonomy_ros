"""奖励函数抽象接口.

设计目标:
- 与模型 / 环境完全解耦, 只输入 rollout 轨迹列表, 输出 [batch_size] 标量 reward.
- 提供 ``DummyRewardFn`` 用于 pipeline 联通测试 (返回零或随机 reward).
- 通过 ``build_reward_fn(config)`` 工厂派发, 后续按任务 (导航成功率 / VLN / 真机
  人工标注 / Reward Model) 扩展具体实现.
"""

from typing import Any, Dict, List, Optional, Union

import torch


class RewardFn:
    """奖励函数基类.

    所有具体奖励函数需实现 ``compute`` 方法, 输入 rollout 轨迹列表, 输出形状为
    ``[batch_size]`` 的标量 reward (float tensor). 训练 trainer 在每个 rollout
    迭代结束后调用一次本对象.
    """

    name: str = "base"

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}

    def reset(self) -> None:
        """每个 rollout 开始前重置内部状态 (可选)."""

    def __call__(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """允许将奖励函数实例当作可调用对象使用, 等价于 compute()."""
        return self.compute(trajectories, **kwargs)

    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        """计算 batch reward.

        Args:
            trajectories: rollout 轨迹列表, 每个元素至少包含
                {"actions": Tensor, "obs": dict, ...}; 具体字段由 trainer 约定.
            **kwargs: 任务相关额外信息 (如 goal, success flag).

        Returns:
            torch.Tensor: 形状 [batch_size] 的 float reward.
        """
        raise NotImplementedError(
            f"{self.__class__.__name__}.compute 未实现, 请在子类中提供具体奖励逻辑."
        )


class DummyRewardFn(RewardFn):
    """占位实现: 返回固定值 / 随机值. 用于 GRPO pipeline 端到端联通测试.

    config:
        mode: "zero" | "random" | "constant" (默认 "zero").
        value: constant 模式下的常数值 (默认 0.0).
        random_std: random 模式下的高斯标准差 (默认 1.0, 均值 0).
    """

    name: str = "dummy"

    def compute(self, trajectories: List[Dict[str, Any]], **kwargs: Any) -> torch.Tensor:
        batch_size = len(trajectories)
        device = kwargs.get("device", torch.device("cpu"))
        dtype = kwargs.get("dtype", torch.float32)

        mode = self.config.get("mode", "zero")
        if mode == "zero":
            return torch.zeros(batch_size, device=device, dtype=dtype)
        if mode == "constant":
            value = float(self.config.get("value", 0.0))
            return torch.full((batch_size,), value, device=device, dtype=dtype)
        if mode == "random":
            std = float(self.config.get("random_std", 1.0))
            return torch.randn(batch_size, device=device, dtype=dtype) * std
        raise ValueError(f"Unknown DummyRewardFn mode: {mode}")


# 工厂注册表 (允许业务侧通过 register_reward_fn 注册自定义实现).
_REWARD_REGISTRY: Dict[str, type] = {"dummy": DummyRewardFn}


def register_reward_fn(name: str):
    """装饰器: 注册自定义奖励函数到全局 registry."""

    def _decorator(cls: type) -> type:
        if name in _REWARD_REGISTRY:
            raise ValueError(f"Reward `{name}` already registered.")
        _REWARD_REGISTRY[name] = cls
        return cls

    return _decorator


def build_reward_fn(
    config_or_name: Union[str, Dict[str, Any]],
    config: Optional[Dict[str, Any]] = None,
) -> "RewardFn":
    """根据配置构造 RewardFn.

    支持两种调用方式:
        1. build_reward_fn({"type": "composite", ...})    # 单 dict 参数
        2. build_reward_fn("composite", {"weight": 1.0}) # (name, config) 双参数

    Args:
        config_or_name: 包含 ``type`` 字段的 config dict, 或奖励函数名称字符串.
        config: 当第一个参数为字符串时, 此 dict 作为奖励函数配置.

    Returns:
        RewardFn 实例.
    """
    if isinstance(config_or_name, str):
        cfg = dict(config or {})
        cfg["type"] = config_or_name
    else:
        cfg = dict(config_or_name or {})

    reward_type = cfg.pop("type", "dummy")
    if reward_type not in _REWARD_REGISTRY:
        raise KeyError(
            f"未注册的 reward type=`{reward_type}`, 可选: {list(_REWARD_REGISTRY.keys())}"
        )
    return _REWARD_REGISTRY[reward_type](config=cfg)