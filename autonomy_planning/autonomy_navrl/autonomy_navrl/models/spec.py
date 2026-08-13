"""Model configuration parsed from training YAML."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class EncoderSpec:
    """Single-modality encoder settings."""

    kind: str
    params: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, kind: str, raw: dict[str, Any] | None) -> EncoderSpec:
        return cls(kind=kind, params=dict(raw or {}))


@dataclass(frozen=True)
class ModelSpec:
    """Policy + multimodal encoder layout."""

    policy_type: str = 'gaussian_actor_critic'
    fusion: str = 'concat'
    action_dim: int = 3
    hidden_dim: int = 256
    state_dim: int = 20
    image_height: int = 96
    image_width: int = 128
    rgbd_channels: int = 4
    cnn_channels: tuple[int, ...] = (32, 64, 64)
    cnn_kernel_sizes: tuple[int, ...] = (8, 4, 3)
    cnn_strides: tuple[int, ...] = (4, 2, 1)
    encoders: dict[str, EncoderSpec] = field(default_factory=dict)

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> ModelSpec:
        """Build from ``model:`` block, falling back to legacy ``policy:`` + ``sensors:``."""
        model_raw = dict(config.get('model', {}))
        policy_raw = dict(config.get('policy', {}))
        sensor_raw = config.get('sensors', {})
        rgb_cfg = sensor_raw.get('rgb', {})

        policy_type = str(model_raw.get('policy_type') or policy_raw.get('type', 'gaussian_actor_critic'))
        fusion = str(model_raw.get('fusion', 'concat'))
        action_dim = int(model_raw.get('action_dim', policy_raw.get('action_dim', 3)))

        encoders_raw = model_raw.get('encoders') or {
            'rgbd': {
                'type': 'cnn',
                'channels': policy_raw.get('cnn_channels', [32, 64, 64]),
                'kernel_sizes': policy_raw.get('cnn_kernel_sizes', [8, 4, 3]),
                'strides': policy_raw.get('cnn_strides', [4, 2, 1]),
            },
            'state': {'type': 'mlp'},
        }
        encoders = {
            name: EncoderSpec.from_dict(
                str(spec.get('type', name)),
                spec if isinstance(spec, dict) else {},
            )
            for name, spec in encoders_raw.items()
        }

        rgb_ch = int(policy_raw.get('rgb_channels', 3))
        depth_ch = int(policy_raw.get('depth_channels', 1))
        rgbd_channels = int(model_raw.get('rgbd_channels', rgb_ch + depth_ch))

        return cls(
            policy_type=policy_type,
            fusion=fusion,
            action_dim=action_dim,
            hidden_dim=int(model_raw.get('hidden_dim', policy_raw.get('hidden_dim', 256))),
            state_dim=int(model_raw.get('state_dim', policy_raw.get('state_dim', 20))),
            image_height=int(rgb_cfg.get('height', model_raw.get('image_height', 96))),
            image_width=int(rgb_cfg.get('width', model_raw.get('image_width', 128))),
            rgbd_channels=rgbd_channels,
            cnn_channels=tuple(encoders.get('rgbd', EncoderSpec('cnn')).params.get(
                'channels', policy_raw.get('cnn_channels', [32, 64, 64])
            )),
            cnn_kernel_sizes=tuple(encoders.get('rgbd', EncoderSpec('cnn')).params.get(
                'kernel_sizes', policy_raw.get('cnn_kernel_sizes', [8, 4, 3])
            )),
            cnn_strides=tuple(encoders.get('rgbd', EncoderSpec('cnn')).params.get(
                'strides', policy_raw.get('cnn_strides', [4, 2, 1])
            )),
            encoders=encoders,
        )
