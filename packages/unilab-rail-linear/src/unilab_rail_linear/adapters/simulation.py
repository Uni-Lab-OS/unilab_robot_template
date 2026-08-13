"""确定性导轨仿真端口，用于无硬件验收 rail-then-arm。"""

from __future__ import annotations

import time
from collections.abc import Callable, MutableSequence

from unilab_robot_contracts import ObservationState, RailStateObservation


class SimulationRailAxisPort:
    """同步完成移动并记录事件的单轴仿真。"""

    def __init__(
        self,
        *,
        endpoint_id: str = "sim:rail",
        events: MutableSequence[str] | None = None,
        on_settled: Callable[[str], None] | None = None,
    ) -> None:
        """创建初始未定位的仿真轴。

        参数：物理端点身份、可选事件记录和到位通知。返回：无。
        到位通知仅连接包内仿真互锁，不构成真实硬件安全证据。
        """

        self.endpoint_ids = frozenset({endpoint_id})
        self.events = events if events is not None else []
        self._on_settled = on_settled
        self._target_ref: str | None = None
        self._completed_command_id: str | None = None
        self._position = 0.0

    def move(self, command_id: str, target_ref: str) -> None:
        """记录开始/完成事件并建立稳定目标。"""

        self.events.append(f"rail:start:{command_id}:{target_ref}")
        self._target_ref = target_ref
        self._completed_command_id = command_id
        self._position += 1.0
        if self._on_settled is not None:
            self._on_settled(command_id)
        self.events.append(f"rail:settled:{command_id}:{target_ref}")

    def observe(self) -> RailStateObservation:
        """返回当前稳定仿真观测。"""

        state = (
            ObservationState.KNOWN
            if self._target_ref is not None
            else ObservationState.UNKNOWN
        )
        return RailStateObservation(
            state,
            time.time(),
            1.0,
            "simulation_rail",
            self._position,
            False,
            True,
            self._target_ref,
            self._completed_command_id,
        )

    def request_stop(self, command_id: str) -> bool:
        """同步仿真总能确认停止。"""

        self.events.append(f"rail:stop:{command_id}")
        return True
