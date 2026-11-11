"""三个 CR5 后端共享的最小状态和末端观测实现。"""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from unilab_robot_contracts import (
    BackendStatus,
    EndEffectorObservation,
    ObservationState,
)


class BackendObservationMixin:
    """为 transport adapter 提供一致状态投影，避免复制公共语义。"""

    endpoint_ids: frozenset[str]

    def _initialize_observation(self) -> None:
        """初始化为空闲在线、末端未知的本地投影。"""

        self._active_command_id: str | None = None
        self._online = True
        self._holding_payload: bool | None = None

    def status(self) -> BackendStatus:
        """返回 adapter 当前连接与执行投影。"""

        return BackendStatus(
            self._online, self._active_command_id is None, self._active_command_id
        )

    def end_effector_observation(self) -> EndEffectorObservation:
        """读取末端负载观测；没有见证时明确返回 unknown。"""

        state = (
            ObservationState.KNOWN
            if self._holding_payload is not None
            else ObservationState.UNKNOWN
        )
        return EndEffectorObservation(
            state, time.time(), 1.0, "cr5_backend", self._holding_payload
        )


def validate_completion_receipt(
    receipt: Mapping[str, Any], *, command_id: str, source: str
) -> Mapping[str, Any]:
    """验证 transport 回执精确绑定命令且明确完成。"""

    if str(receipt.get("command_id", "")) != command_id:
        raise ValueError(f"{source} 回执 command_id 不匹配")
    if str(receipt.get("state", "")) != "succeeded":
        raise ValueError(f"{source} 回执不是 succeeded 终态")
    if receipt.get("completed") is not True:
        raise ValueError(f"{source} 回执缺少 completed=true 见证")
    return receipt
