"""Proximal Policy Optimization."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn

from autonomy_navrl.algorithms.base import BaseAlgorithm
from autonomy_navrl.algorithms.buffers import RolloutBuffer
from autonomy_navrl.core.registry import Registry
from autonomy_navrl.env.base_env import BaseNavrlEnv
from autonomy_navrl.models.checkpoint import load_checkpoint_payload, save_checkpoint
from autonomy_navrl.models.obs import obs_to_torch
from autonomy_navrl.models.policies.base import NavPolicy
from autonomy_navrl.train.logger import TensorboardLogger

algorithm_registry: Registry[type['PpoAlgorithm']] = Registry('algorithm')


@algorithm_registry.register('ppo')
class PpoAlgorithm(BaseAlgorithm):
    """Minimal PPO trainer for vectorized navigation."""

    name = 'ppo'

    def __init__(
        self,
        env: BaseNavrlEnv,
        policy: NavPolicy,
        config: dict[str, Any],
        device: str = 'cuda:0',
    ) -> None:
        super().__init__(env, policy, config, device)
        self._device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self._policy = policy.to(self._device)
        ppo_cfg = config.get('ppo', {})
        self._gamma = float(ppo_cfg.get('gamma', 0.99))
        self._gae_lambda = float(ppo_cfg.get('gae_lambda', 0.95))
        self._clip_range = float(ppo_cfg.get('clip_range', 0.2))
        self._value_coef = float(ppo_cfg.get('value_coef', 0.5))
        self._entropy_coef = float(ppo_cfg.get('entropy_coef', 0.01))
        self._max_grad_norm = float(ppo_cfg.get('max_grad_norm', 0.5))
        self._num_epochs = int(ppo_cfg.get('num_epochs', 4))
        self._num_minibatches = int(ppo_cfg.get('num_minibatches', 4))
        self._rollout_steps = int(ppo_cfg.get('rollout_steps', 128))
        self._total_timesteps = int(ppo_cfg.get('total_timesteps', 1_000_000))
        self._save_interval = int(ppo_cfg.get('save_interval', 10_000))
        self._log_interval = int(ppo_cfg.get('log_interval', 1_000))
        self._learning_rate = float(ppo_cfg.get('learning_rate', 3e-4))
        self._log_images = bool(ppo_cfg.get('log_images', False))
        self._log_image_interval = int(ppo_cfg.get('log_image_interval', 10_000))
        self._resume_path = ppo_cfg.get('resume_checkpoint', '')
        self._global_step = 0
        self._last_rollout_metrics: dict[str, float] = {}

        self._optimizer = torch.optim.Adam(self._policy.parameters(), lr=self._learning_rate)

        run_name = str(config.get('run_name', 'navrl'))
        paths = config.get('paths', {})
        self._checkpoint_dir = Path(str(paths.get('checkpoint_dir', 'checkpoints/%s/ckpts')) % run_name)
        self._checkpoint_dir.mkdir(parents=True, exist_ok=True)
        tb_dir = Path(str(paths.get('tensorboard_dir', 'checkpoints/%s/tensorboard')) % run_name)
        tb_dir.mkdir(parents=True, exist_ok=True)
        self._logger = TensorboardLogger(tb_dir)

        if self._resume_path:
            resume = Path(str(self._resume_path))
            if resume.is_file():
                self._global_step = self.load_checkpoint(resume)
                print(f'[INFO]: Resumed from {resume.resolve()} at step={self._global_step}', flush=True)
            else:
                print(f'[WARNING]: resume checkpoint not found: {resume}', flush=True)

    def train(self) -> Path:
        if self._global_step == 0:
            print(
                '[INFO]: Collecting first rollout (RTX shader compile can take 5-15 min)...',
                flush=True,
            )
        obs = self._env.reset()
        global_step = self._global_step
        while global_step < self._total_timesteps:
            buffer, obs = self._collect_rollout(obs)
            advantages, returns = self._compute_gae(buffer)
            metrics = self._update_policy(buffer, advantages, returns)
            global_step += self._rollout_steps * self._env.num_envs
            self._global_step = global_step
            metrics['global_step'] = float(global_step)
            self._logger.log_dict(global_step, 'train', metrics)
            if self._log_images and global_step % self._log_image_interval < self._rollout_steps * self._env.num_envs:
                self._logger.log_rgbd_preview(global_step, buffer.rgbd[0].detach().cpu())
            if global_step % self._log_interval < self._rollout_steps * self._env.num_envs:
                print(
                    f'[PPO] step={global_step} '
                    f"reward={metrics.get('reward_mean', 0.0):.3f} "
                    f"goal={metrics.get('goal_rate', 0.0):.3f} "
                    f"policy_loss={metrics.get('policy_loss', 0.0):.4f}"
                )
            if global_step % self._save_interval < self._rollout_steps * self._env.num_envs:
                self.save_checkpoint(self._checkpoint_dir / 'navrl_latest.pt')
        self._logger.close()
        return self.save_checkpoint(self._checkpoint_dir / 'navrl_final.pt')

    def load_checkpoint(self, path: Path) -> int:
        payload = load_checkpoint_payload(path, self._device)
        self._policy.load_state_dict(payload['policy_state_dict'])
        if 'optimizer_state_dict' in payload:
            self._optimizer.load_state_dict(payload['optimizer_state_dict'])
        return int(payload.get('global_step', 0))

    def save_checkpoint(self, path: Path) -> Path:
        return save_checkpoint(
            path,
            policy=self._policy,
            optimizer=self._optimizer,
            config=self._config,
            global_step=self._global_step,
            algorithm=self.name,
        )

    def _collect_rollout(
        self,
        obs: dict[str, np.ndarray],
    ) -> tuple[RolloutBuffer, dict[str, np.ndarray]]:
        rgbd_list: list[torch.Tensor] = []
        state_list: list[torch.Tensor] = []
        action_list: list[torch.Tensor] = []
        log_prob_list: list[torch.Tensor] = []
        value_list: list[torch.Tensor] = []
        reward_list: list[torch.Tensor] = []
        done_list: list[torch.Tensor] = []
        rollout_metrics: dict[str, list[float]] = {
            'goal_rate': [],
            'collision_rate': [],
            'progress_mean': [],
        }

        for _ in range(self._rollout_steps):
            obs_t = obs_to_torch(obs, self._device)
            with torch.no_grad():
                action, log_prob, value, _ = self._policy.act(obs_t)
            step = self._env.step(action.detach().cpu().numpy())
            obs = step.observation
            done = np.logical_or(step.terminated, step.truncated)
            step_metrics = step.info.get('metrics', {})
            for key in rollout_metrics:
                if key in step_metrics:
                    rollout_metrics[key].append(float(step_metrics[key]))

            rgbd_list.append(obs_t['rgbd'])
            state_list.append(obs_t['state'])
            action_list.append(action)
            log_prob_list.append(log_prob)
            value_list.append(value)
            reward_list.append(torch.as_tensor(step.reward, device=self._device))
            done_list.append(torch.as_tensor(done.astype(np.float32), device=self._device))

        buffer = RolloutBuffer(
            rgbd=torch.stack(rgbd_list, dim=0),
            state=torch.stack(state_list, dim=0),
            actions=torch.stack(action_list, dim=0),
            log_probs=torch.stack(log_prob_list, dim=0),
            values=torch.stack(value_list, dim=0),
            rewards=torch.stack(reward_list, dim=0),
            dones=torch.stack(done_list, dim=0),
        )
        self._last_rollout_metrics = {
            key: float(np.mean(values)) if values else 0.0
            for key, values in rollout_metrics.items()
        }
        return buffer, obs

    def _compute_gae(self, buffer: RolloutBuffer) -> tuple[torch.Tensor, torch.Tensor]:
        rewards = buffer.rewards
        values = buffer.values
        dones = buffer.dones
        advantages = torch.zeros_like(rewards)
        last_gae = torch.zeros(rewards.shape[1], device=self._device)
        for step in reversed(range(rewards.shape[0])):
            next_value = values[step + 1] if step + 1 < rewards.shape[0] else torch.zeros_like(values[step])
            next_non_terminal = 1.0 - dones[step]
            delta = rewards[step] + self._gamma * next_value * next_non_terminal - values[step]
            last_gae = delta + self._gamma * self._gae_lambda * next_non_terminal * last_gae
            advantages[step] = last_gae
        returns = advantages + values
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        return advantages, returns

    def _update_policy(
        self,
        buffer: RolloutBuffer,
        advantages: torch.Tensor,
        returns: torch.Tensor,
    ) -> dict[str, float]:
        timesteps, num_envs = buffer.rewards.shape
        flat_size = timesteps * num_envs
        obs_flat = buffer.as_obs_dict()
        actions = buffer.actions.reshape(flat_size, buffer.actions.shape[-1])
        old_log_probs = buffer.log_probs.reshape(flat_size)
        advantages_flat = advantages.reshape(flat_size)
        returns_flat = returns.reshape(flat_size)

        batch_size = max(flat_size // self._num_minibatches, 1)
        policy_losses: list[float] = []
        value_losses: list[float] = []
        entropy_values: list[float] = []

        for _ in range(self._num_epochs):
            indices = torch.randperm(flat_size, device=self._device)
            for start in range(0, flat_size, batch_size):
                batch_ids = indices[start:start + batch_size]
                batch_obs = {key: tensor[batch_ids] for key, tensor in obs_flat.items()}
                log_probs, values, entropy = self._policy.evaluate_actions(batch_obs, actions[batch_ids])
                ratio = torch.exp(log_probs - old_log_probs[batch_ids])
                surr1 = ratio * advantages_flat[batch_ids]
                surr2 = torch.clamp(ratio, 1.0 - self._clip_range, 1.0 + self._clip_range)
                surr2 = surr2 * advantages_flat[batch_ids]
                policy_loss = -torch.min(surr1, surr2).mean()
                value_loss = nn.functional.mse_loss(values, returns_flat[batch_ids])
                entropy_bonus = entropy.mean()
                loss = policy_loss + self._value_coef * value_loss - self._entropy_coef * entropy_bonus

                self._optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self._policy.parameters(), self._max_grad_norm)
                self._optimizer.step()

                policy_losses.append(float(policy_loss.detach().cpu()))
                value_losses.append(float(value_loss.detach().cpu()))
                entropy_values.append(float(entropy_bonus.detach().cpu()))

        return {
            'reward_mean': float(buffer.rewards.mean().cpu()),
            'policy_loss': float(np.mean(policy_losses)),
            'value_loss': float(np.mean(value_losses)),
            'entropy': float(np.mean(entropy_values)),
            'goal_rate': float(self._last_rollout_metrics.get('goal_rate', 0.0)),
            'collision_rate': float(self._last_rollout_metrics.get('collision_rate', 0.0)),
            'progress_mean': float(self._last_rollout_metrics.get('progress_mean', 0.0)),
        }
