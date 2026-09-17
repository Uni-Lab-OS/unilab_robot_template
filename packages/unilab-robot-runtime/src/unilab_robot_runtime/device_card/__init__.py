"""机械臂设备卡片（Device Card）template 通用实现。"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .context import ArmCardContext
from .types import (
    RobotDebugSnapshot,
    RobotManualMotionResult,
    RobotRailMotionResult,
    RobotTeachPointResult,
)

if TYPE_CHECKING:
    from .actions_mixin import RailMountedArmCardMixin, RobotQueryResult
    from .preview_card_mixin import PreviewArmCardMixin

__all__ = [
    "ArmCardContext",
    "PreviewArmCardMixin",
    "RailMountedArmCardMixin",
    "RobotDebugSnapshot",
    "RobotManualMotionResult",
    "RobotQueryResult",
    "RobotRailMotionResult",
    "RobotTeachPointResult",
]


def __getattr__(name: str) -> object:
    if name in {"RailMountedArmCardMixin", "RobotQueryResult"}:
        from .actions_mixin import RailMountedArmCardMixin, RobotQueryResult

        return {"RailMountedArmCardMixin": RailMountedArmCardMixin, "RobotQueryResult": RobotQueryResult}[name]
    if name == "PreviewArmCardMixin":
        from .preview_card_mixin import PreviewArmCardMixin

        return PreviewArmCardMixin
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
