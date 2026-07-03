"""Training model registry (aligned with ROS inference policies)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class TrainingSpec:
  """Metadata for a trainable navigation model."""

  model: str
  ros_policy: str
  supported: bool
  description: str
  external_repo: str | None = None
  external_doc: str | None = None


TRAINING_SPECS: dict[str, TrainingSpec] = {
    'navdp': TrainingSpec(
        model='navdp',
        ros_policy='navdp',
        supported=True,
        description='NavDP diffusion policy (InternNav / LeRobot dataset)',
    ),
    'logoplanner': TrainingSpec(
        model='logoplanner',
        ros_policy='logoplanner',
        supported=False,
        description='LoGoPlanner — training not bundled in NavDP',
        external_repo='https://github.com/InternRobotics/NavDP/tree/master/baselines/logoplanner',
        external_doc='https://huggingface.co/InternRobotics/LoGoPlanner',
    ),
    'viplanner': TrainingSpec(
        model='viplanner',
        ros_policy='viplanner',
        supported=False,
        description='VIPlanner — train in upstream ETH repo',
        external_repo='https://github.com/leggedrobotics/viplanner',
    ),
    'vint': TrainingSpec(
        model='vint',
        ros_policy='vint',
        supported=False,
        description='ViNT — train in visualnav-transformer',
        external_repo='https://github.com/robodhruv/visualnav-transformer',
    ),
    'nomad': TrainingSpec(
        model='nomad',
        ros_policy='nomad',
        supported=False,
        description='NoMaD — train in nomad repo (diffusion-policy)',
        external_repo='https://github.com/robodhruv/nomad',
    ),
}

SUPPORTED_MODELS = frozenset(name for name, spec in TRAINING_SPECS.items() if spec.supported)


class UnsupportedTrainingModelError(RuntimeError):
  """Raised when training is requested for a model without a local runner."""


def get_training_spec(model: str) -> TrainingSpec:
  key = model.lower().strip()
  if key not in TRAINING_SPECS:
    supported = ', '.join(sorted(TRAINING_SPECS))
    raise ValueError(f'Unknown training model {model!r}. Known models: {supported}')
  return TRAINING_SPECS[key]


def require_supported_model(model: str) -> TrainingSpec:
  spec = get_training_spec(model)
  if not spec.supported:
    lines = [
        f'Training for {spec.model!r} is not integrated in autonomy_internnav/train yet.',
        spec.description,
    ]
    if spec.external_repo:
      lines.append(f'Upstream: {spec.external_repo}')
    if spec.external_doc:
      lines.append(f'Weights: {spec.external_doc}')
    lines.append(
        'Download pretrained checkpoints into weights/ and use ros2 launch policy:='
        f'{spec.ros_policy}')
    raise UnsupportedTrainingModelError('\n'.join(lines))
  return spec


def list_training_models() -> list[TrainingSpec]:
  return [TRAINING_SPECS[name] for name in sorted(TRAINING_SPECS)]


# Populated by runners package to avoid circular imports.
_RUNNERS: dict[str, Callable] = {}


def register_runner(model: str, runner: Callable) -> None:
  _RUNNERS[model] = runner


def run_training_for_model(model: str, exp_cfg) -> None:
  spec = require_supported_model(model)
  runner = _RUNNERS.get(spec.model)
  if runner is None:
    raise RuntimeError(f'No training runner registered for {spec.model!r}')
  runner(exp_cfg)
