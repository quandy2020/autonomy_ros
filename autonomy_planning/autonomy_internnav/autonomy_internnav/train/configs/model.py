from typing import Optional

from pydantic import BaseModel


class ModelCfg(BaseModel, extra='allow'):
    policy_name: Optional[str] = None
