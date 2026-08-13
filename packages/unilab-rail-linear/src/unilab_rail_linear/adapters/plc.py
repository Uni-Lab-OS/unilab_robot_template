"""配置驱动的单轴导轨 PLC 端口。"""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from unilab_robot_contracts import ObservationState, RailStateObservation


class RailVariablePort(Protocol):
    """真实 PLC 与 PLC-Sim 共享的最小变量端口。"""

    def read(self, variable: str) -> Any:
        """读取一个变量。"""

    def write(self, variable: str, value: Any) -> None:
        """写入一个变量。"""

    def wait_equal(self, variable: str, expected: Any, timeout_s: float) -> bool:
        """等待变量达到期望值。"""


@dataclass(frozen=True)
class PLCRailBinding:
    """导轨原始节点和部署目标值；这些字段不会进入 Graph/RobotCommand。"""

    target_variable: str
    command_id_variable: str
    accepted_command_id_variable: str
    completed_command_id_variable: str
    start_variable: str
    moving_variable: str
    settled_variable: str
    position_variable: str
    target_values: Mapping[str, float]
    timeout_s: float = 60.0
    tolerance: float = 0.001


class PLCRailAxisPort:
    """通过 PLC 节点控制单轴导轨，并投影规范 RailStateObservation。"""

    def __init__(
        self,
        *,
        port: RailVariablePort,
        binding: PLCRailBinding,
        endpoint_ids: frozenset[str],
    ) -> None:
        """注入变量端口、部署绑定和物理 endpoint。"""

        self.port = port
        self.binding = binding
        self.endpoint_ids = endpoint_ids
        self._last_target_ref: str | None = None
        self._last_completed_command_id: str | None = None

    def move(self, command_id: str, target_ref: str) -> None:
        """写入目标和命令身份，再触发一个新周期。"""

        if target_ref not in self.binding.target_values:
            raise ValueError(f"PLC 导轨目标不存在: {target_ref}")
        self.port.write(self.binding.command_id_variable, command_id)
        self.port.write(
            self.binding.target_variable, self.binding.target_values[target_ref]
        )
        self.port.write(self.binding.start_variable, True)
        if not self.port.wait_equal(
            self.binding.accepted_command_id_variable,
            command_id,
            self.binding.timeout_s,
        ):
            raise TimeoutError("等待导轨命令身份确认超时")
        if not self.port.wait_equal(
            self.binding.completed_command_id_variable,
            command_id,
            self.binding.timeout_s,
        ):
            raise TimeoutError("等待导轨本轮完成回执超时")
        if not self.port.wait_equal(
            self.binding.settled_variable, True, self.binding.timeout_s
        ):
            raise TimeoutError("等待导轨 settled 超时")
        self.port.write(self.binding.start_variable, False)
        self._last_target_ref = target_ref
        self._last_completed_command_id = command_id

    def observe(self) -> RailStateObservation:
        """同时读取位置、moving 与 settled，不以单一位置值推断稳定。"""

        try:
            position = float(self.port.read(self.binding.position_variable))
            moving = bool(self.port.read(self.binding.moving_variable))
            settled = bool(self.port.read(self.binding.settled_variable))
        except Exception:  # noqa: BLE001
            return RailStateObservation(
                ObservationState.UNKNOWN, time.time(), 0.5, "plc_rail", None, None, None
            )
        target_ref = self._last_target_ref
        if target_ref is not None:
            target_value = self.binding.target_values[target_ref]
            settled = settled and abs(position - target_value) <= self.binding.tolerance
        return RailStateObservation(
            ObservationState.KNOWN,
            time.time(),
            0.5,
            "plc_rail",
            position,
            moving,
            settled,
            target_ref,
            self._last_completed_command_id,
        )

    def request_stop(self, command_id: str) -> bool:
        """普通映射没有可证明停止的节点，默认返回 False。"""

        return False
