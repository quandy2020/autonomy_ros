# Copyright 2026 autonomy_ros contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""NavDP diffusion policy network."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

_TRAJECTORY_SCALE = 4.0
_SHORT_TRAJECTORY_DIST = 0.5
_NOGOAL_SHORT_PENALTY_DIST = 1.0
_NOGOAL_SHORT_PENALTY = 10.0
_TOP_K = 2
_STOP_YAW = torch.tensor([[[0.0, 0.0, 1.0]]])


class NavDP_Policy(nn.Module):
    """NavDP transformer diffusion policy with critic selection."""

    def __init__(
        self,
        image_size: int = 224,
        memory_size: int = 8,
        predict_size: int = 24,
        temporal_depth: int = 8,
        heads: int = 8,
        token_dim: int = 384,
        channels: int = 3,
        device: str = 'cuda:0',
    ) -> None:
        from autonomy_internnav.baselines.navdp.policy_backbone import (
            LearnablePositionalEncoding,
            NavDP_ImageGoal_Backbone,
            NavDP_PixelGoal_Backbone,
            NavDP_RGBD_Backbone,
            SinusoidalPosEmb,
        )

        super().__init__()
        self.device = device
        self.image_size = image_size
        self.memory_size = memory_size
        self.predict_size = predict_size
        self.temporal_depth = temporal_depth
        self.attention_heads = heads
        self.input_channels = channels
        self.token_dim = token_dim

        self.rgbd_encoder = NavDP_RGBD_Backbone(
            image_size, token_dim, memory_size=memory_size, device=device,
        )
        self.point_encoder = nn.Linear(3, token_dim)
        self.pixel_encoder = NavDP_PixelGoal_Backbone(image_size, token_dim, device=device)
        self.image_encoder = NavDP_ImageGoal_Backbone(image_size, token_dim, device=device)

        decoder_layer = nn.TransformerDecoderLayer(
            d_model=token_dim,
            nhead=heads,
            dim_feedforward=4 * token_dim,
            activation='gelu',
            batch_first=True,
            norm_first=True,
        )
        self.decoder = nn.TransformerDecoder(decoder_layer, num_layers=temporal_depth)

        self.input_embed = nn.Linear(3, token_dim)
        self.cond_pos_embed = LearnablePositionalEncoding(
            token_dim, memory_size * 16 + 4,
        )
        self.out_pos_embed = LearnablePositionalEncoding(token_dim, predict_size)
        self.time_emb = SinusoidalPosEmb(token_dim)
        self.layernorm = nn.LayerNorm(token_dim)
        self.action_head = nn.Linear(token_dim, 3)
        self.critic_head = nn.Linear(token_dim, 1)
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=10,
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon',
        )

        tgt_mask = (torch.triu(torch.ones(predict_size, predict_size)) == 1).transpose(0, 1)
        self.tgt_mask = tgt_mask.float().masked_fill(tgt_mask == 0, float('-inf')).masked_fill(
            tgt_mask == 1, 0.0,
        )
        self.cond_critic_mask = torch.zeros((predict_size, 4 + memory_size * 16))
        self.cond_critic_mask[:, 0:4] = float('-inf')

    def predict_noise(
        self,
        last_actions: torch.Tensor,
        timestep: torch.Tensor,
        goal_embed: torch.Tensor,
        rgbd_embed: torch.Tensor,
    ) -> torch.Tensor:
        action_embeds = self.input_embed(last_actions)
        time_embeds = self.time_emb(timestep.to(self.device)).unsqueeze(1).tile(
            (last_actions.shape[0], 1, 1),
        )
        cond_parts = [time_embeds, goal_embed, goal_embed, goal_embed, rgbd_embed]
        cond_embedding = torch.cat(cond_parts, dim=1) + self.cond_pos_embed(torch.cat(cond_parts, dim=1))
        input_embedding = action_embeds + self.out_pos_embed(action_embeds)
        output = self.decoder(
            tgt=input_embedding,
            memory=cond_embedding,
            tgt_mask=self.tgt_mask.to(self.device),
        )
        return self.action_head(self.layernorm(output))

    def predict_mix_noise(
        self,
        last_actions: torch.Tensor,
        timestep: torch.Tensor,
        goal_embeds: list[torch.Tensor],
        rgbd_embed: torch.Tensor,
    ) -> torch.Tensor:
        action_embeds = self.input_embed(last_actions)
        time_embeds = self.time_emb(timestep.to(self.device)).unsqueeze(1).tile(
            (last_actions.shape[0], 1, 1),
        )
        cond_parts = [time_embeds, goal_embeds[0], goal_embeds[1], goal_embeds[2], rgbd_embed]
        cond_embedding = torch.cat(cond_parts, dim=1) + self.cond_pos_embed(torch.cat(cond_parts, dim=1))
        input_embedding = action_embeds + self.out_pos_embed(action_embeds)
        output = self.decoder(
            tgt=input_embedding,
            memory=cond_embedding,
            tgt_mask=self.tgt_mask.to(self.device),
        )
        return self.action_head(self.layernorm(output))

    def predict_critic(
        self,
        predict_trajectory: torch.Tensor,
        rgbd_embed: torch.Tensor,
    ) -> torch.Tensor:
        nogoal_embed = torch.zeros_like(rgbd_embed[:, 0:1])
        action_embeddings = self.input_embed(predict_trajectory)
        action_embeddings = action_embeddings + self.out_pos_embed(action_embeddings)
        cond_parts = [nogoal_embed, nogoal_embed, nogoal_embed, nogoal_embed, rgbd_embed]
        cond_embeddings = torch.cat(cond_parts, dim=1) + self.cond_pos_embed(
            torch.cat(cond_parts, dim=1),
        )
        critic_output = self.decoder(
            tgt=action_embeddings,
            memory=cond_embeddings,
            memory_mask=self.cond_critic_mask.to(self.device),
        )
        critic_output = self.layernorm(critic_output)
        return self.critic_head(critic_output.mean(dim=1))[:, 0]

    def _denoise_actions(
        self,
        batch_size: int,
        sample_num: int,
        goal_embed: torch.Tensor,
        rgbd_embed: torch.Tensor,
        noise_predictor: Callable[..., torch.Tensor],
        noise_predictor_args: tuple[Any, ...] = (),
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Run DDPM denoising and critic scoring."""
        rgbd_embed = torch.repeat_interleave(rgbd_embed, sample_num, dim=0)
        goal_embed = torch.repeat_interleave(goal_embed, sample_num, dim=0)
        naction = torch.randn(
            (sample_num * batch_size, self.predict_size, 3), device=self.device,
        )
        self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
        for timestep in self.noise_scheduler.timesteps[:]:
            noise_pred = noise_predictor(
                naction, timestep.unsqueeze(0), goal_embed, rgbd_embed, *noise_predictor_args,
            )
            naction = self.noise_scheduler.step(
                model_output=noise_pred, timestep=timestep, sample=naction,
            ).prev_sample
        critic_values = self.predict_critic(naction, rgbd_embed)
        critic_values = critic_values.reshape(batch_size, sample_num)
        return naction, critic_values

    def _actions_to_trajectories(
        self,
        naction: torch.Tensor,
        batch_size: int,
        sample_num: int,
    ) -> torch.Tensor:
        all_trajectory = torch.cumsum(naction / _TRAJECTORY_SCALE, dim=1)
        return all_trajectory.reshape(batch_size, sample_num, self.predict_size, 3)

    def _mask_short_trajectories(self, all_trajectory: torch.Tensor) -> torch.Tensor:
        trajectory_length = all_trajectory[:, :, -1, 0:2].norm(dim=-1)
        stop_yaw = _STOP_YAW.to(all_trajectory.device)
        all_trajectory[trajectory_length < _SHORT_TRAJECTORY_DIST] = (
            all_trajectory[trajectory_length < _SHORT_TRAJECTORY_DIST] * stop_yaw
        )
        return all_trajectory

    def _select_topk_trajectories(
        self,
        all_trajectory: torch.Tensor,
        critic_values: torch.Tensor,
        batch_size: int,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        batch_idx = torch.arange(batch_size).unsqueeze(1).expand(-1, _TOP_K)
        best_idx = (-critic_values).argsort(dim=1)[:, 0:_TOP_K]
        worst_idx = critic_values.argsort(dim=1)[:, 0:_TOP_K]
        positive = all_trajectory[batch_idx, best_idx]
        negative = all_trajectory[batch_idx, worst_idx]
        return positive, negative

    def _finalize_diffusion(
        self,
        naction: torch.Tensor,
        critic_values: torch.Tensor,
        batch_size: int,
        sample_num: int,
        *,
        penalize_short_nogoal: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        all_trajectory = self._actions_to_trajectories(naction, batch_size, sample_num)
        if penalize_short_nogoal:
            trajectory_length = all_trajectory[:, :, -1, 0:2].norm(dim=-1)
            critic_values = critic_values.clone()
            critic_values[torch.where(trajectory_length < _NOGOAL_SHORT_PENALTY_DIST)] -= (
                _NOGOAL_SHORT_PENALTY
            )
        else:
            all_trajectory = self._mask_short_trajectories(all_trajectory)
        positive, negative = self._select_topk_trajectories(
            all_trajectory, critic_values, batch_size,
        )
        return (
            all_trajectory.cpu().numpy(),
            critic_values.cpu().numpy(),
            positive.cpu().numpy(),
            negative.cpu().numpy(),
        )

    def _predict_with_goal_embed(
        self,
        batch_size: int,
        sample_num: int,
        goal_embed: torch.Tensor,
        rgbd_embed: torch.Tensor,
        noise_predictor: Callable[..., torch.Tensor],
        noise_predictor_args: tuple[Any, ...] = (),
        *,
        penalize_short_nogoal: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        naction, critic_values = self._denoise_actions(
            batch_size, sample_num, goal_embed, rgbd_embed,
            noise_predictor, noise_predictor_args,
        )
        return self._finalize_diffusion(
            naction, critic_values, batch_size, sample_num,
            penalize_short_nogoal=penalize_short_nogoal,
        )

    def predict_pointgoal_action(
        self,
        goal_point: np.ndarray,
        input_images: np.ndarray,
        input_depths: np.ndarray,
        sample_num: int = 16,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with torch.no_grad():
            tensor_point_goal = torch.as_tensor(goal_point, dtype=torch.float32, device=self.device)
            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            pointgoal_embed = self.point_encoder(tensor_point_goal).unsqueeze(1)
            return self._predict_with_goal_embed(
                goal_point.shape[0], sample_num, pointgoal_embed, rgbd_embed, self.predict_noise,
            )

    def predict_imagegoal_action(
        self,
        goal_image: np.ndarray,
        input_images: np.ndarray,
        input_depths: np.ndarray,
        sample_num: int = 16,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            imagegoal_embed = self.image_encoder(
                np.concatenate((goal_image, input_images[:, -1]), axis=-1),
            ).unsqueeze(1)
            return self._predict_with_goal_embed(
                goal_image.shape[0], sample_num, imagegoal_embed, rgbd_embed, self.predict_noise,
            )

    def predict_pixelgoal_action(
        self,
        goal_image: np.ndarray,
        input_images: np.ndarray,
        input_depths: np.ndarray,
        sample_num: int = 16,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            pixelgoal_embed = self.pixel_encoder(
                np.concatenate((goal_image[:, :, :, None], input_images[:, -1]), axis=-1),
            ).unsqueeze(1)
            return self._predict_with_goal_embed(
                goal_image.shape[0], sample_num, pixelgoal_embed, rgbd_embed, self.predict_noise,
            )

    def predict_nogoal_action(
        self,
        input_images: np.ndarray,
        input_depths: np.ndarray,
        sample_num: int = 16,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with torch.no_grad():
            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            nogoal_embed = torch.zeros_like(rgbd_embed[:, 0:1])
            return self._predict_with_goal_embed(
                input_images.shape[0],
                sample_num,
                nogoal_embed,
                rgbd_embed,
                self.predict_noise,
                penalize_short_nogoal=True,
            )

    def predict_ip_action(
        self,
        goal_point: np.ndarray,
        goal_image: np.ndarray,
        input_images: np.ndarray,
        input_depths: np.ndarray,
        sample_num: int = 16,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with torch.no_grad():
            batch_size = goal_image.shape[0]
            tensor_point_goal = torch.as_tensor(goal_point, dtype=torch.float32, device=self.device)
            rgbd_embed = self.rgbd_encoder(input_images, input_depths)
            imagegoal_embed = self.image_encoder(
                np.concatenate((goal_image, input_images[:, -1]), axis=-1),
            ).unsqueeze(1)
            pointgoal_embed = self.point_encoder(tensor_point_goal).unsqueeze(1)

            rgbd_embed = torch.repeat_interleave(rgbd_embed, sample_num, dim=0)
            pointgoal_embed = torch.repeat_interleave(pointgoal_embed, sample_num, dim=0)
            imagegoal_embed = torch.repeat_interleave(imagegoal_embed, sample_num, dim=0)
            mix_goal_embeds = [imagegoal_embed, pointgoal_embed, imagegoal_embed]

            naction = torch.randn(
                (sample_num * batch_size, self.predict_size, 3), device=self.device,
            )
            self.noise_scheduler.set_timesteps(self.noise_scheduler.config.num_train_timesteps)
            for timestep in self.noise_scheduler.timesteps[:]:
                noise_pred = self.predict_mix_noise(
                    naction, timestep.unsqueeze(0), mix_goal_embeds, rgbd_embed,
                )
                naction = self.noise_scheduler.step(
                    model_output=noise_pred, timestep=timestep, sample=naction,
                ).prev_sample
            critic_values = self.predict_critic(naction, rgbd_embed).reshape(batch_size, sample_num)
            return self._finalize_diffusion(
                naction, critic_values, batch_size, sample_num,
            )
