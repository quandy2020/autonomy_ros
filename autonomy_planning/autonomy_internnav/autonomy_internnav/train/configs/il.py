from typing import Optional

from pydantic import BaseModel


class Loss(BaseModel, extra='allow'):
    alpha: Optional[float] = None
    dist_scale: Optional[int] = None


class FilterFailure(BaseModel, extra='allow'):
    use: Optional[bool] = None
    min_rgb_nums: Optional[int] = None


class IlCfg(BaseModel, extra='allow'):
    epochs: Optional[int] = None
    batch_size: Optional[int] = None
    lr: Optional[float] = None
    num_workers: Optional[int] = None
    weight_decay: Optional[float] = None
    warmup_ratio: Optional[float] = None
    use_iw: Optional[bool] = None
    inflection_weight_coef: Optional[float] = None
    save_interval_epochs: Optional[int] = None
    save_interval_steps: Optional[int] = None
    save_filter_frozen_weights: Optional[bool] = None
    load_from_ckpt: Optional[bool] = None
    ckpt_to_load: Optional[str] = None
    lmdb_map_size: Optional[float] = None
    dataset_r2r_root_dir: Optional[str] = None
    dataset_3dgs_root_dir: Optional[str] = None
    dataset_grutopia10_root_dir: Optional[str] = None
    lmdb_features_dir: Optional[str] = None
    lerobot_features_dir: Optional[str] = None
    camera_name: Optional[str] = None
    filter_failure: Optional[FilterFailure] = None
    use_discrete_dataset: Optional[bool] = None
    loss: Optional[Loss] = None
    report_to: Optional[str] = None
    # NavDP-specific
    dataset_navdp: Optional[str] = None
    root_dir: Optional[str] = None
    image_size: Optional[int] = None
    scene_scale: Optional[float] = None
    preload: Optional[bool] = None
    random_digit: Optional[bool] = None
    prior_sample: Optional[bool] = None
    memory_size: Optional[int] = None
    predict_size: Optional[int] = None
    pixel_channel: Optional[int] = None
    temporal_depth: Optional[int] = None
    heads: Optional[int] = None
    token_dim: Optional[int] = None
    channels: Optional[int] = None
    dropout: Optional[float] = None
    scratch: Optional[bool] = None
    finetune: Optional[bool] = None
    ddp_find_unused_parameters: Optional[bool] = None
    dataset_repeat: Optional[int] = None
    dataset_format: Optional[str] = None
    gradient_accumulation_steps: Optional[int] = None
    save_total_limit: Optional[int] = None
    obstacle_sample_n: Optional[int] = None
    use_pointcloud: Optional[bool] = None
    sample_interval: Optional[int] = None
    dataset_sources: Optional[list] = None
    mixed_epoch_size: Optional[int] = None
    target_segment_weights: Optional[list] = None
