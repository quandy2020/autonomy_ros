"""RGB-D convolutional encoder."""

from __future__ import annotations

from typing import Sequence

import torch
from torch import nn


class RgbdEncoder(nn.Module):
    """Encode stacked RGB-D images to a flat feature vector."""

    def __init__(
        self,
        input_channels: int,
        image_height: int,
        image_width: int,
        channels: Sequence[int],
        kernel_sizes: Sequence[int],
        strides: Sequence[int],
    ) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        in_ch = input_channels
        for out_ch, kernel, stride in zip(channels, kernel_sizes, strides, strict=True):
            layers.extend([
                nn.Conv2d(in_ch, out_ch, kernel, stride=stride),
                nn.ReLU(inplace=True),
            ])
            in_ch = out_ch
        self._cnn = nn.Sequential(*layers)
        self.output_dim = self._infer_flatten_dim(input_channels, image_height, image_width)

    def _infer_flatten_dim(self, input_channels: int, height: int, width: int) -> int:
        with torch.no_grad():
            dummy = torch.zeros(1, input_channels, height, width)
            return int(self._cnn(dummy).view(1, -1).shape[1])

    def forward(self, rgbd: torch.Tensor) -> torch.Tensor:
        return self._cnn(rgbd).view(rgbd.shape[0], -1)
