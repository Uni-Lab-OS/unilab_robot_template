"""导轨先到位、硬件切换许可、再动机械臂的唯一协调状态机。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from threading import RLock

from unilab_robot_contracts import (
    ArmModulePort,
    CommandJournal,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DeploymentMode,
    DispatchUnknownError,
    HardwareProfile,
    PhysicalSettlementEvidence,
    RobotCommand,
    RailModulePort,
    SafetyInterlockObservation,
)


class RailMountedArmCoordinator:
    """一个公共命令内严格串行协调导轨和机械臂，并保留 UNKNOWN Fence。"""

    def __init__(
        self,
        *,
        arm: ArmModulePort,
        rail: RailModulePort,
        profile: HardwareProfile,
        journal: CommandJournal,
        safety_observation: Callable[[], SafetyInterlockObservation],
    ) -> None:
        """注入可替换 Arm/Rail 模块和部署级权威依赖。

        参数：模块端点必须互不重叠且全部被 profile 锁定。返回：无。异常：装配错误时拒绝启动。
        """

        if arm.endpoint_ids.intersection(rail.endpoint_ids):
            raise ValueError("ArmModule 与 RailModule 不得声明同一物理端点")
        endpoints = arm.endpoint_ids.union(rail.endpoint_ids)
        if not endpoints.issubset(profile.endpoint_ids):
            raise ValueError("WorkCell 模块端点未被 HardwareProfile 原子锁定")
        self.arm = arm
        self.rail = rail
        self.profile = profile
        self.journal = journal
        self._safety_observation = safety_observation
        self._execution_lock = RLock()
        self._phase_lock = RLock()
        self._active_phases: dict[str, str] = {}

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回 WorkCell 独占的 Arm 与 Rail 端点并集。"""

        return self.arm.endpoint_ids.union(self.rail.endpoint_ids)

    @property
    def has_unsettled_fence(self) -> bool:
        """公共复合命令或私有机械臂任一未结算时返回 True。"""

        return bool(self.journal.fenced_command_ids()) or self.arm.has_unsettled_fence

    def execute(self, command: RobotCommand, *, rail_target_ref: str) -> CommandResult:
        """执行 rail-then-arm，并在任何派发后歧义上保留 Claim/Fence 语义。

        参数：公共 RobotCommand 与独立导轨目标。返回：同一公共 command_id 的结果。异常：无；错误收敛为 rejected 或 execution_unknown。
        """

        if command.hardware_profile_digest != self.profile.digest:
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                "WorkCell profile digest 不匹配",
            )
        if self.journal.get(command.command_id) is None:
            fenced = self.journal.fenced_command_ids()
            if fenced:
                return CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    f"WorkCell 存在未物理结算命令，禁止新派发: {fenced}",
                )
        try:
            created, existing = self.journal.accept(command)
        except CommandRejectedError as exc:
            return CommandResult(command.command_id, CommandState.REJECTED, str(exc))
        if not created:
            return existing
        if rail_target_ref not in self.rail.allowed_targets:
            return self.journal.update(
                CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    f"导轨 target-set 不包含: {rail_target_ref}",
                )
            )
        if self.arm.has_unsettled_fence:
            return self.journal.update(
                CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    "机械臂私有账本存在未结算 Fence，导轨不得先行移动",
                )
            )

        with self._execution_lock:
            interrupted = self._interrupted_result(command.command_id)
            if interrupted is not None:
                return interrupted
            before = self._rail_phase_rejection()
            if before:
                return self.journal.update(
                    CommandResult(command.command_id, CommandState.REJECTED, before)
                )
            running = self._mark_running(command.command_id, "RAIL_MOVING")
            if running.state is not CommandState.RUNNING:
                return running
            self._set_active_phase(command.command_id, "rail")
            try:
                interrupted = self._interrupted_result(command.command_id)
                if interrupted is not None:
                    return interrupted
                rail_observation = self.rail.move_and_settle(
                    command.command_id, rail_target_ref
                )
            except Exception as exc:  # noqa: BLE001
                detail = (
                    str(exc)
                    if isinstance(exc, DispatchUnknownError)
                    else f"未分类派发后异常: {exc}"
                )
                return self._contain_unknown(
                    command.command_id,
                    reason="导轨阶段结果不明",
                    detail=detail,
                    phase="rail",
                )
            finally:
                self._clear_active_phase(command.command_id, "rail")

            interrupted = self._interrupted_result(command.command_id)
            if interrupted is not None:
                return interrupted

            after = self._arm_phase_rejection()
            if after:
                return self._contain_unknown(
                    command.command_id,
                    reason="导轨到位后安全许可切换失败",
                    detail=after,
                    phase="between",
                )
            running = self._mark_running(command.command_id, "ARM_MOVING")
            if running.state is not CommandState.RUNNING:
                return running
            private_arm_command = replace(
                command, command_id=f"{command.command_id}:arm"
            )
            self._set_active_phase(command.command_id, "arm")
            try:
                interrupted = self._interrupted_result(command.command_id)
                if interrupted is not None:
                    return interrupted
                arm_result = self.arm.execute(private_arm_command)
            except Exception as exc:  # noqa: BLE001
                return self._contain_unknown(
                    command.command_id,
                    reason="机械臂阶段结果不明",
                    detail=f"未分类派发后异常: {exc}",
                    phase="arm",
                )
            finally:
                self._clear_active_phase(command.command_id, "arm")

            interrupted = self._interrupted_result(command.command_id)
            if interrupted is not None:
                return interrupted
            if arm_result.state is not CommandState.SUCCEEDED:
                if arm_result.state is CommandState.EXECUTION_UNKNOWN:
                    return self._contain_unknown(
                        command.command_id,
                        reason="机械臂阶段结果不明",
                        detail=arm_result.message,
                        phase="arm",
                    )
                return self.journal.update(
                    CommandResult(
                        command.command_id,
                        CommandState.FAILED,
                        f"机械臂阶段未结算: {arm_result.message}",
                    ),
                    fenced=False,
                )
            return self._finish_if_not_interrupted(
                CommandResult(
                    command.command_id,
                    CommandState.SUCCEEDED,
                    "RAIL_SETTLED → ARM_COMPLETED",
                    {"rail_target_ref": rail_observation.target_ref},
                )
            )

    def request_controlled_stop(
        self, command_id: str, *, reason: str
    ) -> CommandResult:
        """锁存 UNKNOWN 后请求 Arm/Rail 普通停止，不宣称硬件急停。

        参数：已接受的公共命令身份与操作原因。返回：仍为
        ``execution_unknown`` 的公共结果。异常：无；停止通道错误写入诊断。
        """

        existing = self.journal.get(command_id)
        if existing is None:
            return CommandResult(
                command_id,
                CommandState.REJECTED,
                "无法停止未被 WorkCell 接受的命令",
            )
        if existing.state.terminal and not self.journal.is_fenced(command_id):
            return existing
        phase = self._active_phase(command_id)
        return self._contain_unknown(
            command_id,
            reason=reason,
            detail="外部请求了软件侧应急遏制",
            phase=phase,
        )

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
        *,
        arm_result: CommandResult | None = None,
        arm_evidence: PhysicalSettlementEvidence | None = None,
    ) -> CommandResult:
        """用组合设备物理见证显式解除本地派发阻断。

        参数：公共结果/见证，以及机械臂私有账本被阻断时必需的私有
        结果/见证。返回：已结算公共结果。异常：见证缺失、身份不匹配或
        执行仍未退出时拒绝；该接口不触发任何物理动作。
        """

        with self._execution_lock:
            if result.command_id != evidence.command_id:
                raise CommandRejectedError("WorkCell 物理结算见证 command_id 不匹配")
            if self.arm.has_unsettled_fence:
                if arm_result is None or arm_evidence is None:
                    raise CommandRejectedError(
                        "机械臂私有命令仍未结算，必须同时提供私有物理见证"
                    )
                expected_arm_id = f"{result.command_id}:arm"
                if (
                    arm_result.command_id != expected_arm_id
                    or arm_evidence.command_id != expected_arm_id
                ):
                    raise CommandRejectedError(
                        "机械臂私有物理见证未绑定当前 WorkCell 命令"
                    )
                self.arm.settle_unknown(arm_result, arm_evidence)
            return self.journal.settle(result, evidence)

    def _mark_running(self, command_id: str, phase: str) -> CommandResult:
        """原子写入运行阶段；若已被停止请求锁存则返回当前 UNKNOWN。"""

        try:
            return self.journal.update(
                CommandResult(command_id, CommandState.RUNNING, phase),
                fenced=True,
            )
        except CommandRejectedError:
            existing = self.journal.get(command_id)
            if existing is None:
                raise
            return existing

    def _finish_if_not_interrupted(self, result: CommandResult) -> CommandResult:
        """只在没有并发停止锁存时提交成功终态。"""

        interrupted = self._interrupted_result(result.command_id)
        if interrupted is not None:
            return interrupted
        try:
            return self.journal.update(result, fenced=False)
        except CommandRejectedError:
            existing = self.journal.get(result.command_id)
            if existing is None:
                raise
            return existing

    def _interrupted_result(self, command_id: str) -> CommandResult | None:
        """返回由应急遏制锁存的 UNKNOWN；其他状态返回 None。"""

        existing = self.journal.get(command_id)
        if (
            existing is not None
            and existing.state is CommandState.EXECUTION_UNKNOWN
            and self.journal.is_fenced(command_id)
        ):
            return existing
        return None

    def _contain_unknown(
        self,
        command_id: str,
        *,
        reason: str,
        detail: str,
        phase: str | None,
    ) -> CommandResult:
        """先持久阻断，再分别请求两模块停止并保存诊断结果。"""

        initial = CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            f"{reason}: {detail}",
            {
                "controlled_stop": {
                    "status": "requesting",
                    "physical_settlement": False,
                }
            },
        )
        try:
            self.journal.update(initial, fenced=True)
        except CommandRejectedError:
            existing = self.journal.get(command_id)
            if existing is None or not self.journal.is_fenced(command_id):
                return existing or initial

        report = self._request_module_stops(command_id, reason=reason, phase=phase)
        result = CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            f"{reason}: {detail}；已请求普通受控停止，仍需可信物理结算",
            {"controlled_stop": report},
        )
        try:
            return self.journal.update(result, fenced=True)
        except CommandRejectedError:
            existing = self.journal.get(command_id)
            return existing or result

    def _request_module_stops(
        self, command_id: str, *, reason: str, phase: str | None
    ) -> dict[str, object]:
        """优先停止当前相位，再停止另一模块；所有错误只进入诊断。"""

        report: dict[str, object] = {
            "phase": phase or "unknown",
            "physical_settlement": False,
        }

        def stop_arm() -> None:
            """请求机械臂停止并规范化诊断字段。"""

            try:
                result = self.arm.request_controlled_stop(
                    f"{command_id}:arm", reason
                )
                report["arm"] = {
                    "requested": True,
                    "confirmed": result.state is CommandState.CANCELED,
                    "state": result.state.value,
                    "message": result.message,
                }
            except Exception as exc:  # noqa: BLE001
                report["arm"] = {
                    "requested": True,
                    "confirmed": False,
                    "error": str(exc),
                }

        def stop_rail() -> None:
            """请求导轨停止并规范化诊断字段。"""

            try:
                confirmed = self.rail.request_controlled_stop(command_id, reason)
                report["rail"] = {
                    "requested": True,
                    "confirmed": bool(confirmed),
                }
            except Exception as exc:  # noqa: BLE001
                report["rail"] = {
                    "requested": True,
                    "confirmed": False,
                    "error": str(exc),
                }

        if phase == "arm":
            stop_arm()
            stop_rail()
        else:
            stop_rail()
            stop_arm()
        return report

    def _set_active_phase(self, command_id: str, phase: str) -> None:
        """记录当前可被外部停止请求观察的派发相位。"""

        with self._phase_lock:
            self._active_phases[command_id] = phase

    def _clear_active_phase(self, command_id: str, phase: str) -> None:
        """仅清除仍匹配的活动相位，避免覆盖并发更新。"""

        with self._phase_lock:
            if self._active_phases.get(command_id) == phase:
                self._active_phases.pop(command_id, None)

    def _active_phase(self, command_id: str) -> str | None:
        """返回命令当前活动相位；跨重启未知时返回 None。"""

        with self._phase_lock:
            return self._active_phases.get(command_id)

    def _rail_phase_rejection(self) -> str | None:
        """验证开始移动导轨前硬件必须禁止机械臂运动。"""

        try:
            observed = self._safety_observation()
        except Exception as exc:  # noqa: BLE001
            return f"硬件安全观测读取失败: {exc}"
        common = self._common_safety_rejection(observed)
        if common:
            return common
        if not observed.rail_motion_permitted:
            return "硬件安全链未许可导轨运动"
        if observed.arm_motion_permitted:
            return "导轨阶段硬件仍允许机械臂运动，拒绝危险并发窗口"
        return None

    def _arm_phase_rejection(self) -> str | None:
        """验证导轨稳定后硬件已切换到只允许机械臂运动。"""

        try:
            observed = self._safety_observation()
        except Exception as exc:  # noqa: BLE001
            return f"硬件安全观测读取失败: {exc}"
        common = self._common_safety_rejection(observed)
        if common:
            return common
        if not observed.arm_motion_permitted:
            return "导轨到位后硬件未许可机械臂运动"
        if observed.rail_motion_permitted:
            return "机械臂阶段硬件仍允许导轨运动，保持 execution_unknown"
        return None

    def _common_safety_rejection(
        self, observed: SafetyInterlockObservation
    ) -> str | None:
        """验证安全观测新鲜、grant、互斥能力与生产级硬件证据。"""

        if not observed.is_fresh():
            return "硬件安全观测未知或过期"
        if not observed.granted or not observed.concurrent_motion_blocked:
            return "硬件安全链未建立运动互斥许可"
        if (
            self.profile.mode is DeploymentMode.PRODUCTION
            and not observed.hardware_enforced
        ):
            return "production profile 缺少经验证硬件互锁"
        return None
