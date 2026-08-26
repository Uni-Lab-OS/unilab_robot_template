"""机械臂、导轨与组合协调器之间的最小模块端口。"""

from __future__ import annotations

from typing import Protocol

from .commands import CommandResult, PhysicalSettlementEvidence, RobotCommand
from .observations import RailStateObservation


class ArmModulePort(Protocol):
    """组合层所需的机械臂能力；不绑定型号或传输方式。"""

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回机械臂物理端点。"""

    @property
    def has_unsettled_fence(self) -> bool:
        """是否存在未物理结算的本地派发阻断。"""

    def fenced_command_ids(self) -> tuple[str, ...]:
        """返回机械臂私有账本中全部未结算命令身份。"""

    def execute(self, command: RobotCommand) -> CommandResult:
        """执行已解析的机械臂命令。"""

    def validate_before_dispatch(self, command: RobotCommand) -> None:
        """在组合设备任何轴运动前完成无物理作用预校验。"""

    def request_controlled_stop(
        self, command_id: str, reason: str
    ) -> CommandResult:
        """请求普通受控停止；返回值只用于诊断，不代表物理结算。"""

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
    ) -> CommandResult:
        """以精确物理见证显式结算机械臂 UNKNOWN。"""


class RailModulePort(Protocol):
    """组合层所需的单轴导轨能力；不绑定型号或 PLC/SDK。"""

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回导轨物理端点。"""

    @property
    def allowed_targets(self) -> frozenset[str]:
        """返回部署已发布的目标引用。"""

    def move_and_settle(
        self, command_id: str, target_ref: str
    ) -> RailStateObservation:
        """移动并返回精确绑定命令的稳定见证。"""

    def observe(self) -> RailStateObservation:
        """读取导轨观测。"""

    def request_controlled_stop(self, command_id: str, reason: str) -> bool:
        """请求普通受控停止；确认值不得用于解除 UNKNOWN 阻断。"""
