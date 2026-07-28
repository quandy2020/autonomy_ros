"""Extended model config schema for InternVLA-N1 (JdInternNav base_encoders subset)."""

from typing import Any, Dict, Optional

from pydantic import BaseModel


class ModelCfg(BaseModel, extra='allow'):
    policy_name: Optional[str] = None
    model_settings: Optional[Dict[str, Any]] = None
