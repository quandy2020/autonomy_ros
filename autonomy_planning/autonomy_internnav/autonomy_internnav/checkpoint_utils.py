"""Checkpoint loading helpers aligned with JdInternNav navdp_policy."""

from __future__ import annotations

import os

import torch


def load_ckpt_state(path: str) -> dict:
    """Load model weights from an HF checkpoint directory or a single file.

    Supports: model.safetensors, pytorch_model.bin, navdp.ckpt, plain .pt/.bin
    """
    if os.path.isdir(path):
        for fname in ('model.safetensors', 'pytorch_model.bin', 'navdp.ckpt'):
            full = os.path.join(path, fname)
            if not os.path.exists(full):
                continue
            if fname.endswith('.safetensors'):
                from safetensors.torch import load_file

                return load_file(full)
            return torch.load(full, map_location='cpu', weights_only=False)
        raise FileNotFoundError(f'No supported checkpoint file in {path}')
    return torch.load(path, map_location='cpu', weights_only=False)
