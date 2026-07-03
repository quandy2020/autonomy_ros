from typing import List, Optional

from pydantic import BaseModel

from autonomy_internnav.train.configs.eval import EvalCfg
from autonomy_internnav.train.configs.il import IlCfg
from autonomy_internnav.train.configs.model import ModelCfg


class ExpCfg(BaseModel, extra='allow'):
    name: Optional[str] = None
    model_name: Optional[str] = None
    torch_gpu_id: Optional[int] = None
    torch_gpu_ids: Optional[List[int]] = None
    checkpoint_folder: Optional[str] = None
    tensorboard_dir: Optional[str] = None
    output_dir: Optional[str] = None
    log_dir: Optional[str] = None
    local_rank: Optional[int] = None
    num_gpus: Optional[int] = None
    world_size: Optional[int] = None
    seed: Optional[int] = None
    eval: Optional[EvalCfg] = None
    il: Optional[IlCfg] = None
    model: Optional[ModelCfg] = None
