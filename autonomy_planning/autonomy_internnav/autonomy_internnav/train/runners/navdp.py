"""NavDP training runner."""

from __future__ import annotations

import logging
import os
import sys
import traceback
from datetime import datetime
from pathlib import Path

import torch
import torch.distributed as dist
from transformers import TrainerCallback, TrainingArguments

from autonomy_internnav.train.dataset import NavDP_Base_Datset, navdp_collate_fn
from autonomy_internnav.train.dataset_lerobot_v3 import NavDP_LerobotV3_Dataset
from autonomy_internnav.train.dataset_mixed import NavDPMixedSourceDataset
from autonomy_internnav.train.logger import MyLogger
from autonomy_internnav.train.navdp_model import NavDPModelConfig, NavDPNet
from autonomy_internnav.train.navdp_trainer import NavDPTrainer


def _resolve_report_to(report_to):
  if not report_to or str(report_to).lower() == 'none':
    return 'none'
  if report_to != 'tensorboard':
    return report_to
  try:
    import tensorboard  # noqa: F401
  except ImportError:
    print(
        'tensorboard is not installed in the training venv; '
        'disabling TensorBoard logging. Install with:\n'
        f'  {sys.executable} -m pip install "tensorboard>=2.20.0,<3.0"\n'
        'or run: bash scripts/install_train_deps.sh'
    )
    return 'none'
  return report_to


class CheckpointFormatCallback(TrainerCallback):
  """Placeholder callback for checkpoint post-processing."""

  def __init__(self, run_name: str, exp_cfg_dir: Path | None = None):
    self.exp_cfg_dir = exp_cfg_dir

  def on_save(self, args, state, control, **kwargs):
    if state.is_world_process_zero:
      _ = Path(args.output_dir) / f'checkpoint-{state.global_step}'


def _make_dir(config):
  config.tensorboard_dir = config.tensorboard_dir % config.name
  config.checkpoint_folder = config.checkpoint_folder % config.name
  config.log_dir = config.log_dir % config.name
  config.output_dir = config.output_dir % config.name
  for path in (config.tensorboard_dir, config.checkpoint_folder, config.log_dir):
    os.makedirs(path, exist_ok=True)


def _normalize_navdp_dataset_format(dataset_format):
  aliases = {
      'v2': 'v2',
      'navdp_v1': 'v2',
      'v3': 'v3',
      'lerobot_v3': 'v3',
      'mixed': 'mixed',
      'mixed_navdp': 'mixed',
  }
  normalized = aliases.get(str(dataset_format))
  if normalized is None:
    raise ValueError(
        f"Unknown config.il.dataset_format={dataset_format!r}; "
        "expected 'v2', 'v3', or 'mixed'")
  return normalized


def _build_navdp_dataset(config):
  dataset_format = _normalize_navdp_dataset_format(
      getattr(config.il, 'dataset_format', 'v2') or 'v2')
  use_pointcloud = (
      True if getattr(config.il, 'use_pointcloud', None) is None
      else bool(config.il.use_pointcloud))
  print(f'dataset_format: {dataset_format}')
  if not use_pointcloud:
    print('use_pointcloud=False: SFT 不使用障碍物点云')
  if dataset_format == 'mixed':
    return NavDPMixedSourceDataset(
        getattr(config.il, 'dataset_sources', None),
        epoch_size=getattr(config.il, 'mixed_epoch_size', None),
        memory_size=config.il.memory_size,
        predict_size=config.il.predict_size,
        batch_size=config.il.batch_size,
        image_size=config.il.image_size,
        pixel_channel=config.il.pixel_channel,
        sample_interval=config.il.sample_interval or 4,
        use_pointcloud=use_pointcloud,
    )
  if dataset_format == 'v3':
    return NavDP_LerobotV3_Dataset(
        config.il.root_dir,
        memory_size=config.il.memory_size,
        predict_size=config.il.predict_size,
        batch_size=config.il.batch_size,
        image_size=config.il.image_size,
        pixel_channel=config.il.pixel_channel,
        sample_interval=config.il.sample_interval or 4,
        target_segment_weights=getattr(config.il, 'target_segment_weights', None),
        use_pointcloud=use_pointcloud,
    )
  return NavDP_Base_Datset(
      config.il.root_dir,
      config.il.dataset_navdp,
      config.il.memory_size,
      config.il.predict_size,
      config.il.batch_size,
      config.il.image_size,
      config.il.scene_scale,
      pixel_channel=config.il.pixel_channel,
      preload=config.il.preload,
      random_digit=config.il.random_digit,
      prior_sample=config.il.prior_sample,
      dataset_repeat=config.il.dataset_repeat or 1,
      obstacle_sample_n=config.il.obstacle_sample_n or 2048,
      sample_interval=config.il.sample_interval or 4,
      target_segment_weights=getattr(config.il, 'target_segment_weights', None),
      use_pointcloud=use_pointcloud,
  )


def run_navdp_training(config) -> None:
  try:
    _make_dir(config)

    print('=== Start NavDP training ===')
    print(f'Current time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    print(f'PyTorch version: {torch.__version__}')
    print(f'CUDA available: {torch.cuda.is_available()}')
    print(f'CUDA device count: {torch.cuda.device_count()}')

    local_rank = int(os.getenv('LOCAL_RANK', '0'))
    world_size = int(os.getenv('WORLD_SIZE', '1'))
    rank = int(os.getenv('RANK', '0'))

    device_id = local_rank
    torch.cuda.set_device(device_id)
    device = torch.device(f'cuda:{device_id}')
    print(f'World size: {world_size}, Local rank: {local_rank}, Global rank: {rank}')

    if world_size > 1:
      try:
        dist.init_process_group(
            backend='nccl', init_method='env://', world_size=world_size, rank=rank)
        print('Distributed initialization SUCCESS')
      except Exception as exc:
        print(f'Distributed initialization FAILED: {exc}')
        world_size = 1

    model_cfg = NavDPModelConfig(model_cfg=config.model_dump())
    ckpt_path = config.il.ckpt_to_load or None
    if ckpt_path:
      print(f'load model from: {ckpt_path}')
    model = NavDPNet.from_pretrained(pretrained_model_name_or_path=ckpt_path, config=model_cfg)
    model.to(device)

    for name, param in model.named_parameters():
      if 'mask_token' in name:
        param.requires_grad = False

    for name, param in model.named_parameters():
      if param.device != device:
        print(f'Parameter {name} on {param.device}, moving to {device}')
        param.data = param.data.to(device)

    for name, buffer in model.named_buffers():
      if buffer.device != device:
        print(f'Buffer {name} on {buffer.device}, moving to {device}')
        buffer.data = buffer.data.to(device)

    if world_size > 1:
      model = torch.nn.parallel.DistributedDataParallel(
          model,
          device_ids=[local_rank],
          output_device=local_rank,
          find_unused_parameters=config.il.ddp_find_unused_parameters,
      )

    train_logger_filename = os.path.join(config.log_dir, 'train.log')
    if dist.is_initialized() and dist.get_rank() == 0:
      train_logger = MyLogger(
          name='train',
          level=logging.INFO,
          format_str='%(asctime)-15s %(message)s',
          filename=train_logger_filename,
      )
    else:
      train_logger = MyLogger(
          name='train', level=logging.INFO, format_str='%(asctime)-15s %(message)s')

    transformers_logger = logging.getLogger('transformers')
    if transformers_logger.hasHandlers():
      transformers_logger.handlers = []
    if local_rank in (0, -1):
      transformers_logger.addHandler(train_logger.handlers[0])
    transformers_logger.setLevel(logging.INFO)

    train_dataset = _build_navdp_dataset(config)

    grad_accum = config.il.gradient_accumulation_steps or 1

    training_args = TrainingArguments(
        output_dir=config.output_dir,
        run_name=config.name,
        remove_unused_columns=False,
        deepspeed='',
        gradient_checkpointing=False,
        bf16=False,
        tf32=False,
        per_device_train_batch_size=config.il.batch_size,
        gradient_accumulation_steps=grad_accum,
        dataloader_num_workers=config.il.num_workers,
        dataloader_pin_memory=False,
        optim='adamw_torch',
        learning_rate=config.il.lr,
        lr_scheduler_type='cosine',
        logging_steps=10.0,
        num_train_epochs=config.il.epochs,
        save_strategy='epoch',
        save_steps=config.il.save_interval_epochs,
        save_total_limit=8,
        report_to=_resolve_report_to(config.il.report_to),
        seed=0,
        do_eval=False,
        ddp_find_unused_parameters=config.il.ddp_find_unused_parameters,
        ddp_bucket_cap_mb=100,
        torch_compile_mode=None,
        dataloader_drop_last=True,
        disable_tqdm=True,
        log_level='info',
    )

    trainer = NavDPTrainer(
        config=config,
        model=model,
        args=training_args,
        train_dataset=train_dataset,
        data_collator=navdp_collate_fn,
    )

    trainer.add_callback(
        CheckpointFormatCallback(run_name=config.name, exp_cfg_dir=config.log_dir))

    trainer.train()
    if train_logger:
      for handler in train_logger.handlers:
        handler.flush()

  except Exception:
    traceback.print_exc()
    if dist.is_initialized():
      dist.destroy_process_group()
    raise
