"""不依赖 PLC-Sim 扩展节点的组合设备仿真互锁与生命周期包装。"""

from __future__ import annotations

import time
from threading import RLock

from unilab_robot_contracts import (
    CommandResult,
    ObservationState,
    PhysicalSettlementEvidence,
    RailMoveCommand,
    RobotCommand,
    SafetyInterlockObservation,
)

from .coordinator import RailMountedArmCoordinator


class SimulationInterlockProvider:
    """只用于仿真的 rail-ready/arm-ready 互斥许可状态机。"""

    def __init__(self) -> None:
        """创建初始只允许导轨运动的仿真许可；不产生硬件证据。"""

        self._lock = RLock()
        self._phase = "rail"
        self._active_command_id: str | None = None

    def rail_settled(self, command_id: str) -> None:
        """在同一组合命令导轨到位后切换为只允许机械臂运动。

        参数：本次公共组合命令身份。返回：无。异常：另一命令尚未结束时
        拒绝覆盖其仿真许可，避免测试掩盖并发错误。
        """

        with self._lock:
            if self._active_command_id not in {None, command_id}:
                raise RuntimeError("仿真互锁已有另一活动组合命令")
            self._active_command_id = command_id
            self._phase = "arm"

    def reset(self, command_id: str) -> None:
        """在命令完成物理结算后恢复为只允许导轨运动。

        参数：已结算公共命令身份。返回：无。身份不匹配时保持关闭失败。
        """

        with self._lock:
            if self._active_command_id not in {None, command_id}:
                return
            self._active_command_id = None
            self._phase = "rail"

    def read(self) -> SafetyInterlockObservation:
        """返回新鲜的仿真互斥许可；返回值永远不是硬件强制证据。"""

        with self._lock:
            rail_permitted = self._phase == "rail"
            arm_permitted = self._phase == "arm"
        return SafetyInterlockObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "simulation-only:package-local-workcell",
            True,
            False,
            rail_permitted,
            arm_permitted,
            True,
        )


class SimulationRailMountedArmRuntime:
    """在一个小接口后封装仿真许可周期，复用正式协调状态机。"""

    def __init__(
        self,
        *,
        coordinator: RailMountedArmCoordinator,
        interlock: SimulationInterlockProvider,
    ) -> None:
        """注入正式协调器与包内仿真互锁；返回组合运行时。"""

        self._coordinator = coordinator
        self._interlock = interlock

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回组合设备原子占用的机械臂与仿真导轨端点。"""

        return self._coordinator.endpoint_ids

    @property
    def has_unsettled_fence(self) -> bool:
        """返回是否仍存在未完成物理结算的本地派发阻断。"""

        return self._coordinator.has_unsettled_fence

    def fenced_command_ids(self) -> tuple[str, ...]:
        """返回控制面可对账的公共 Fence 身份。"""

        return self._coordinator.fenced_command_ids()

    def get_command(self, command_id: str) -> CommandResult | None:
        """只读返回公共命令投影。"""

        return self._coordinator.get_command(command_id)

    def resolve_unknown_as_canceled(
        self,
        command_id: str,
        *,
        witness_id: str,
        reason: str,
        source: str,
    ) -> CommandResult:
        """结算同一 Coordinator 的 UNKNOWN，并在清栅栏后复位仿真许可。"""

        result = self._coordinator.resolve_unknown_as_canceled(
            command_id,
            witness_id=witness_id,
            reason=reason,
            source=source,
        )
        if not self._coordinator.has_unsettled_fence:
            self._interlock.reset(command_id)
        return result

    def execute(self, command: RobotCommand, *, rail_target_ref: str) -> CommandResult:
        """执行正式 rail-then-arm 流程，并在已结算终态后复位仿真许可。"""

        result = self._coordinator.execute(command, rail_target_ref=rail_target_ref)
        if not self._coordinator.has_unsettled_fence:
            self._interlock.reset(command.command_id)
        return result

    def move_rail(self, command: RailMoveCommand) -> CommandResult:
        """执行同一 Coordinator 的 rail-only 流程并复位仿真许可。"""

        result = self._coordinator.move_rail(command)
        if not self._coordinator.has_unsettled_fence:
            self._interlock.reset(command.command_id)
        return result

    def request_controlled_stop(
        self, command_id: str, *, reason: str
    ) -> CommandResult:
        """请求正式协调器遏制两端；UNKNOWN 结算前不复位仿真许可。"""

        return self._coordinator.request_controlled_stop(command_id, reason=reason)

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
        *,
        arm_result: CommandResult | None = None,
        arm_evidence: PhysicalSettlementEvidence | None = None,
    ) -> CommandResult:
        """以可信见证结算 UNKNOWN，并仅在全部阻断清除后复位仿真许可。"""

        settled = self._coordinator.settle_unknown(
            result,
            evidence,
            arm_result=arm_result,
            arm_evidence=arm_evidence,
        )
        if not self._coordinator.has_unsettled_fence:
            self._interlock.reset(result.command_id)
        return settled
