"""回放 runner 子包。子模块惰性导入，避免离线无 torch 时无法 import api/converter。"""
__all__ = [
    "NavDPModel", "InferenceResult",
    "CriticSelector",
    "WorldFrameConverter",
    "ReportBuilder",
    "ReplayRunner",
]


def __getattr__(name):
    if name in ("NavDPModel", "InferenceResult"):
        from .model import NavDPModel, InferenceResult
        return NavDPModel if name == "NavDPModel" else InferenceResult
    if name == "CriticSelector":
        from .selector import CriticSelector
        return CriticSelector
    if name == "WorldFrameConverter":
        from .converter import WorldFrameConverter
        return WorldFrameConverter
    if name == "ReportBuilder":
        from .report import ReportBuilder
        return ReportBuilder
    if name == "ReplayRunner":
        from .replay_runner import ReplayRunner
        return ReplayRunner
    raise AttributeError(name)