"""按 device_id 注册 preview 机械臂实例，供领域 visual 联动。"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .preview_arm_device import PreviewArmDevice

LOCAL_ARMS: dict[str, PreviewArmDevice] = {}
