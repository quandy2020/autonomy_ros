"""Export trained S10 locomotion policy to TorchScript for autonomy_navrl s10_jit."""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument('--task', type=str, default='Isaac-Velocity-Rough-S10-v0')
parser.add_argument('--load_run', type=str, required=True, help='Run directory or path relative to log root')
parser.add_argument('--checkpoint', type=str, default='model_*.pt')
parser.add_argument('--output', type=str, required=True)
parser.add_argument('--num_envs', type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import s10_locomotion.register_env  # noqa: F401
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, export_policy_as_jit
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config
from rsl_rl.runners import OnPolicyRunner


def _resolve_checkpoint(load_run: str, checkpoint: str, experiment_name: str) -> str:
    if os.path.isfile(load_run):
        return load_run
    if os.path.isdir(load_run):
        matches = sorted(glob.glob(os.path.join(load_run, checkpoint)))
        if not matches:
            raise FileNotFoundError(f'no checkpoint matching {checkpoint} in {load_run}')
        return matches[-1]
    log_root = os.path.join('logs', 'rsl_rl', experiment_name)
    return get_checkpoint_path(log_root, load_run, checkpoint)


@hydra_task_config(args_cli.task, 'rsl_rl_cfg_entry_point')
def main(env_cfg, agent_cfg):
    resume_path = _resolve_checkpoint(args_cli.load_run, args_cli.checkpoint, agent_cfg.experiment_name)

    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs

    env = gym.make(args_cli.task, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)

    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    export_dir = os.path.join(os.path.dirname(resume_path), 'exported')
    export_policy_as_jit(policy_nn, runner.obs_normalizer, path=export_dir, filename='policy.pt')

    os.makedirs(os.path.dirname(args_cli.output), exist_ok=True)
    shutil.copy2(os.path.join(export_dir, 'policy.pt'), args_cli.output)
    print(f'[export] saved {args_cli.output}')

    env.close()


if __name__ == '__main__':
    # hydra_task_config expects sys.argv to only contain script name
    sys.argv = [sys.argv[0]]
    main()
    simulation_app.close()
