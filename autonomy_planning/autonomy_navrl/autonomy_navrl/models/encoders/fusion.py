"""Multimodal fusion encoders."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

import torch
from torch import nn

from autonomy_navrl.core.registry import Registry
from autonomy_navrl.models.spec import ModelSpec

fusion_registry: Registry[type['BaseFusionEncoder']] = Registry('fusion')


class BaseFusionEncoder(nn.Module, ABC):
    """Fuse multiple observation modalities into a fixed-size embedding."""

    name: str
    output_dim: int

    @abstractmethod
    def forward(self, obs: dict[str, torch.Tensor]) -> torch.Tensor:
        """``obs`` keys depend on fusion type (e.g. rgbd, state)."""


@fusion_registry.register('concat')
class ConcatFusionEncoder(BaseFusionEncoder):
    """Early fusion: CNN(rgbd) + MLP(state) → concat → MLP.

    Submodule names (``_cnn``, ``_state_mlp``, ``_fusion``) match legacy
    ``RgbdStateEncoder`` for checkpoint compatibility.
    """

    name = 'concat'

    def __init__(
        self,
        input_channels: int | None = None,
        image_height: int | None = None,
        image_width: int | None = None,
        cnn_channels: Sequence[int] | None = None,
        cnn_kernel_sizes: Sequence[int] | None = None,
        cnn_strides: Sequence[int] | None = None,
        state_dim: int | None = None,
        hidden_dim: int | None = None,
        *,
        spec: ModelSpec | None = None,
    ) -> None:
        super().__init__()
        if spec is not None:
            input_channels = spec.rgbd_channels
            image_height = spec.image_height
            image_width = spec.image_width
            cnn_channels = spec.cnn_channels
            cnn_kernel_sizes = spec.cnn_kernel_sizes
            cnn_strides = spec.cnn_strides
            state_dim = spec.state_dim
            hidden_dim = spec.hidden_dim

        missing = [
            name
            for name, value in (
                ('input_channels', input_channels),
                ('image_height', image_height),
                ('image_width', image_width),
                ('cnn_channels', cnn_channels),
                ('cnn_kernel_sizes', cnn_kernel_sizes),
                ('cnn_strides', cnn_strides),
                ('state_dim', state_dim),
                ('hidden_dim', hidden_dim),
            )
            if value is None
        ]
        if missing:
            raise TypeError(
                'ConcatFusionEncoder requires spec= or all of: '
                + ', '.join(missing)
            )

        layers: list[nn.Module] = []
        in_channels = input_channels
        for out_channels, kernel, stride in zip(cnn_channels, cnn_kernel_sizes, cnn_strides, strict=True):
            layers.extend([
                nn.Conv2d(in_channels, out_channels, kernel, stride=stride),
                nn.ReLU(inplace=True),
            ])
            in_channels = out_channels
        self._cnn = nn.Sequential(*layers)
        cnn_out = self._infer_flatten_dim(input_channels, image_height, image_width)
        self._state_mlp = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self._fusion = nn.Sequential(
            nn.Linear(cnn_out + hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        self.output_dim = hidden_dim

    def _infer_flatten_dim(self, input_channels: int, height: int, width: int) -> int:
        with torch.no_grad():
            dummy = torch.zeros(1, input_channels, height, width)
            return int(self._cnn(dummy).view(1, -1).shape[1])

    def forward(self, obs: dict[str, torch.Tensor]) -> torch.Tensor:
        rgbd = obs['rgbd']
        state = obs['state']
        image_features = self._cnn(rgbd).view(rgbd.shape[0], -1)
        state_features = self._state_mlp(state)
        return self._fusion(torch.cat([image_features, state_features], dim=-1))


def create_fusion_encoder(spec: ModelSpec) -> BaseFusionEncoder:
    return fusion_registry.create(spec.fusion, spec=spec)
