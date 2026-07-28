#!/usr/bin/env python3
"""Unified training entry point for autonomy_internnav models."""

import torch
import tyro
from pydantic import BaseModel

# Register built-in runners on import.
import autonomy_internnav.train.runners  # noqa: F401
from autonomy_internnav.train.configs.navdp_default import navdp_exp_cfg
from autonomy_internnav.train.registry import (
    SUPPORTED_MODELS,
    UnsupportedTrainingModelError,
    get_training_spec,
    require_supported_model,
    run_training_for_model,
)


class TrainCfg(BaseModel):
  """Training CLI configuration."""

  name: str = 'navdp_train'
  model: str = 'navdp'
  profile: str = 'default'


def _build_exp_cfg(model: str, run_name: str, profile: str):
  spec = get_training_spec(model)
  if spec.model == 'navdp':
    if profile in ('local', 'navdp-local', 'navdp_local'):
      from autonomy_internnav.train.configs.navdp_local import navdp_local_exp_cfg
      exp_cfg = navdp_local_exp_cfg.model_copy(deep=True)
    else:
      exp_cfg = navdp_exp_cfg.model_copy(deep=True)
  elif spec.model == 'navdp_grpo':
    raise RuntimeError(
        'Use scripts/train/train_grpo.py or launch_grpo_train.sh for GRPO training')
  else:
    raise RuntimeError(f'No experiment config for model {spec.model!r}')
  exp_cfg.name = run_name
  exp_cfg.model_name = spec.model
  exp_cfg.num_gpus = len(exp_cfg.torch_gpu_ids)
  exp_cfg.world_size = exp_cfg.num_gpus
  return exp_cfg


def main():
  cli_cfg = tyro.cli(TrainCfg)
  model = cli_cfg.model.lower().strip()

  print('\n' + '=' * 50)
  print('INTERNNAV TRAINING CONFIGURATION:')
  print('=' * 50)
  for key, value in vars(cli_cfg).items():
    print(f'{key}: {value}')
  print('=' * 50 + '\n')

  spec = get_training_spec(model)
  print(f'Model: {spec.model} — {spec.description}')

  try:
    require_supported_model(model)
  except UnsupportedTrainingModelError as exc:
    print(str(exc))
    raise SystemExit(1) from exc

  exp_cfg = _build_exp_cfg(model, cli_cfg.name, cli_cfg.profile)

  available_gpus = torch.cuda.device_count() if torch.cuda.is_available() else 1
  assert exp_cfg.num_gpus <= available_gpus, (
      f'Requested {exp_cfg.num_gpus} GPUs but only {available_gpus} available')
  assert exp_cfg.num_gpus > 0, 'Number of GPUs must be greater than 0'
  print(f'Using {exp_cfg.num_gpus} GPU(s); supported local trainers: {sorted(SUPPORTED_MODELS)}')

  run_training_for_model(model, exp_cfg)


if __name__ == '__main__':
  main()
