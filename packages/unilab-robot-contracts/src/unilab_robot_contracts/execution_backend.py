"""PLC、TCP/SDK 与 MoveIt 共用的执行后端端口。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .commands import CommandResult, RobotCommand
from .observations import EndEffectorObservation


@dataclass(frozen=True)
class BackendStatus:
    """后端最小状态；不把厂商私有字段提升为公共合同。"""

    online: bool
    idle: bool
    active_command_id: str | None = None
    diagnostic: str = ""


class RobotExecutionBackend(Protocol):
    """所有机械臂控制类型必须实现的单一执行生命周期。"""

    endpoint_ids: frozenset[str]

    def status(self) -> BackendStatus:
        """读取连接、空闲和当前命令状态。"""

    def execute(self, command: RobotCommand) -> CommandResult:
        """只执行已由部署解析的命令；不得解析 Site 或 Material。"""

    def reconcile(self, command_id: str) -> CommandResult:
        """对账已派发命令，不得由此自动重放物理动作。"""

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """请求普通受控停止；无法确认停止时返回 execution_unknown。"""

    def end_effector_observation(self) -> EndEffectorObservation:
        """读取末端执行器观测，未知或过期时由上层关闭失败。"""
