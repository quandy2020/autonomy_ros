from typing import Any, Dict, Optional

from pydantic import BaseModel


class AgentCfg(BaseModel):
    server_host: str = 'localhost'
    server_port: int = 8087
    model_name: str
    ckpt_path: Optional[str] = None
    model_settings: Dict[str, Any] = {}
