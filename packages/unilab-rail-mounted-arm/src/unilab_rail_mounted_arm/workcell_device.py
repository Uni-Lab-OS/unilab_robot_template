"""导轨机械臂唯一公共 Device 的框架无关薄包装。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from unilab_robot_contracts import (
    CommandResult,
    PhysicalSettlementEvidence,
    RobotCommand,
)

from .coordinator import RailMountedArmCoordinator
from .endpoint_guard import EndpointLease, EndpointLeaseRegistry


class RailMountedArmWorkCellDevice:
    """公开 pick/place/pour；内部 Arm/Rail 永不成为第二公共 Device。"""

    def __init__(
        self,
        *,
        device_id: str,
        coordinator: RailMountedArmCoordinator,
        resolve_action: Callable[..., tuple[RobotCommand, str]],
        endpoint_registry: EndpointLeaseRegistry,
    ) -> None:
        """激活唯一端点租约并注入部署动作解析器。"""

        self._coordinator = coordinator
        self._resolve_action = resolve_action
        self._lease: EndpointLease = endpoint_registry.acquire(
            device_id, coordinator.endpoint_ids
        )

    def close(self) -> None:
        """释放公共 Device 的物理端点租约。"""

        self._lease.close()

    def request_controlled_stop(
        self, command_id: str, reason: str
    ) -> CommandResult:
        """请求组合设备软件侧应急遏制，不新增 FE 业务动作。"""

        return self._coordinator.request_controlled_stop(command_id, reason=reason)

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
        *,
        arm_result: CommandResult | None = None,
        arm_evidence: PhysicalSettlementEvidence | None = None,
    ) -> CommandResult:
        """透出维护结算入口，不把它注册为 FE 业务动作。"""

        return self._coordinator.settle_unknown(
            result,
            evidence,
            arm_result=arm_result,
            arm_evidence=arm_evidence,
        )

    def pick(self, **arguments: Any) -> CommandResult:
        """执行厂商无关 pick；部署解析器决定导轨和机械臂资产。"""

        command, rail_target = self._resolve_action("pick", **arguments)
        return self._coordinator.execute(command, rail_target_ref=rail_target)

    def place(self, **arguments: Any) -> CommandResult:
        """执行厂商无关 place。"""

        command, rail_target = self._resolve_action("place", **arguments)
        return self._coordinator.execute(command, rail_target_ref=rail_target)

    def pour(self, **arguments: Any) -> CommandResult:
        """执行厂商无关 pour，翻转保持为内部 MotionSegment。"""

        command, rail_target = self._resolve_action("pour", **arguments)
        return self._coordinator.execute(command, rail_target_ref=rail_target)
