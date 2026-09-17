"""Preview 段执行协议；L3 注入 target_ref 解析与运动实现。"""

from __future__ import annotations

import threading
from typing import Protocol, runtime_checkable

from unilab_robot_contracts import MotionSegment

from .preview_arm_device import PreviewArmDevice


@runtime_checkable
class PreviewSegmentExecutor(Protocol):
    """把已解析 MotionSegment 转为 PreviewArmDevice 上的本地运动。"""

    def execute_arm_move(
        self,
        device: PreviewArmDevice,
        segment: MotionSegment,
        *,
        stop_event: threading.Event,
        duration_s: float,
    ) -> None:
        """执行单个 arm_move 段；失败时抛出异常。"""
