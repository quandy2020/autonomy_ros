import os
import threading
import time

import torch
import torch.distributed as dist
from torch.utils.data import DataLoader, DistributedSampler

from autonomy_internnav.train.base_trainer import BaseTrainer


class NavDPTrainer(BaseTrainer):
    def __init__(self, config, **kwargs):
        super().__init__(**kwargs)
        self.config = config
        self.writer = None
        self._last_loss_metrics = {}
        self.start_time = time.time()
        if hasattr(self.model, 'module'):
            self.model_device = self.model.module.device
        else:
            self.model_device = self.model.device

        print(f"[Rank {dist.get_rank() if dist.is_initialized() else 0}] Model device: {self.model_device}")

    def compute_loss(self, model, inputs, return_outputs=False, num_items_in_batch=None):
        model_device = next(model.parameters()).device

        inputs_on_device = {
            key: value.to(model_device, non_blocking=True) if torch.is_tensor(value) else value
            for key, value in inputs.items()
        }

        batch_label_critic = inputs_on_device['batch_label_critic']
        batch_augment_critic = inputs_on_device['batch_augment_critic']

        pred_ng, pred_mg, critic_pred, augment_pred, ng_noise, mg_noise, imagegoal_aux_pred, pixelgoal_aux_pred = model(
            inputs_on_device['batch_pg'],
            inputs_on_device['batch_ig'],
            inputs_on_device['batch_tg'],
            inputs_on_device['batch_rgb'],
            inputs_on_device['batch_depth'],
            inputs_on_device['batch_labels'],
            inputs_on_device['batch_augments'],
        )

        ng_action_loss = (pred_ng - ng_noise).square().mean()
        mg_action_loss = (pred_mg - mg_noise).square().mean()
        aux_loss = (
            0.5 * (inputs_on_device['batch_pg'] - imagegoal_aux_pred).square().mean()
            + 0.5 * (inputs_on_device['batch_pg'] - pixelgoal_aux_pred).square().mean()
        )
        action_loss = 0.5 * mg_action_loss + 0.5 * ng_action_loss
        critic_loss = (critic_pred - batch_label_critic).square().mean() + (
            augment_pred - batch_augment_critic
        ).square().mean()
        loss = 0.8 * action_loss + 0.2 * critic_loss + 0.5 * aux_loss
        self._last_loss_metrics = {
            'loss/total': loss.detach().item(),
            'loss/action': action_loss.detach().item(),
            'loss/ng_action': ng_action_loss.detach().item(),
            'loss/mg_action': mg_action_loss.detach().item(),
            'loss/critic': critic_loss.detach().item(),
            'loss/aux': aux_loss.detach().item(),
        }

        outputs = {
            'pred_ng': pred_ng,
            'pred_mg': pred_mg,
            'critic_pred': critic_pred,
            'augment_pred': augment_pred,
            'noise': [ng_noise, mg_noise],
            'loss': loss,
            'ng_action_loss': ng_action_loss,
            'mg_action_loss': mg_action_loss,
            'action_loss': action_loss,
            'aux_loss': aux_loss,
            'critic_loss': critic_loss,
        }

        return (loss, outputs) if return_outputs else loss

    def create_optimizer(self):
        rank = dist.get_rank() if dist.is_initialized() else 0

        try:
            lr = self.config.il.lr
            if rank == 0:
                print(f'[Rank 0] Using learning rate: {lr}')
        except AttributeError:
            lr = 1e-4
            if rank == 0:
                print(f'[Rank 0] Warning: Using default learning rate: {lr}')

        if hasattr(self.model, 'module'):
            model_for_optim = self.model.module
        else:
            model_for_optim = self.model

        optimizer = torch.optim.Adam(model_for_optim.parameters(), lr=lr)

        if rank == 0:
            print(f'[Rank 0] Optimizer created with {len(optimizer.param_groups)} param groups')
            total_params = sum(p.numel() for p in model_for_optim.parameters() if p.requires_grad)
            print(f'[Rank 0] Total trainable parameters: {total_params:,}')

        return optimizer

    def create_scheduler(self, optimizer, num_training_steps: int):
        if dist.is_initialized():
            rank = dist.get_rank()
        else:
            rank = 0
        if rank == 0:
            print(
                '[Rank 0] LR scheduler: '
                'LinearLR(start_factor=1.0, end_factor=0.5, total_iters=10000); '
                f'estimated_total_steps={num_training_steps}'
            )
        scheduler = torch.optim.lr_scheduler.LinearLR(
            optimizer, start_factor=1.0, end_factor=0.5, total_iters=10000
        )
        return scheduler

    def create_optimizer_and_scheduler(self, num_training_steps: int):
        print('\n=== create optimizer and scheduler ===')
        self.optimizer = self.create_optimizer()
        self.lr_scheduler = self.create_scheduler(self.optimizer, num_training_steps)
        return self.optimizer, self.lr_scheduler

    def get_train_dataloader(self):
        world_size = dist.get_world_size() if dist.is_initialized() else 1
        rank = dist.get_rank() if dist.is_initialized() else 0
        sampler = DistributedSampler(
            self.train_dataset, num_replicas=world_size, rank=rank, shuffle=True, seed=1234
        )

        batch_size = self.config.il.batch_size
        dataset_len = len(self.train_dataset)
        # Small local datasets (e.g. 5 episodes) must not drop the only partial
        # batch, otherwise len(dataloader)==0 and HF Trainer requires max_steps.
        drop_last = dataset_len >= batch_size * world_size

        loader = DataLoader(
            self.train_dataset,
            batch_size=batch_size,
            sampler=sampler,
            num_workers=self.config.il.num_workers,
            pin_memory=True,
            drop_last=drop_last,
            collate_fn=self.data_collator,
        )
        return loader

    def save_model(self, output_dir, state_dict=None, **kwargs):
        model_to_save = self.model
        if hasattr(model_to_save, 'module'):
            model_to_save = model_to_save.module
        if hasattr(model_to_save, '_orig_mod'):
            model_to_save = model_to_save._orig_mod
        os.makedirs(output_dir, exist_ok=True)
        cpu_state_dict = {k: v.cpu().clone() for k, v in model_to_save.state_dict().items()}
        print(f'Saving model to {output_dir} (async)')

        def _async_save():
            try:
                torch.save(cpu_state_dict, os.path.join(output_dir, 'navdp.ckpt'))
                model_to_save.save_pretrained(output_dir, state_dict=cpu_state_dict)
                print(f'Save complete: {output_dir}')
            except Exception as exc:
                print(f'Async save failed for {output_dir}: {exc}')

        if hasattr(self, '_save_thread') and self._save_thread.is_alive():
            self._save_thread.join()
        self._save_thread = threading.Thread(target=_async_save, daemon=True)
        self._save_thread.start()

    def log(self, logs, start_time=None):
        logs = dict(logs)
        logs.update(self._last_loss_metrics)
        if self.optimizer is not None and len(self.optimizer.param_groups) > 0:
            logs['lr'] = self.optimizer.param_groups[0]['lr']

        if start_time is None:
            return super().log(logs)
        return super().log(logs, start_time=start_time)
