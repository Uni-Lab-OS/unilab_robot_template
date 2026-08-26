"""把 CR5 后端生命周期收敛为一个可复用的深模块。"""

from __future__ import annotations

from collections.abc import Callable

from unilab_robot_contracts import (
    CommandJournal,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DeploymentMode,
    DispatchUnknownError,
    HardwareProfile,
    PhysicalSettlementEvidence,
    RobotCommand,
    RobotExecutionBackend,
    SafetyInterlockObservation,
    settle_unknown_as_canceled,
)


class ArmModule:
    """机械臂执行模块；standalone 与 WorkCell 复用同一个实例接口。"""

    def __init__(
        self,
        *,
        backend: RobotExecutionBackend,
        profile: HardwareProfile,
        journal: CommandJournal,
        safety_observation: Callable[[], SafetyInterlockObservation],
    ) -> None:
        """绑定后端、部署配置、命令账本与只读安全观测。

        参数：四个依赖均由部署组合根注入。返回：无。异常：端点或 profile 不一致时拒绝启动。
        """

        if not backend.endpoint_ids:
            raise ValueError("机械臂后端必须声明物理 endpoint_ids")
        if not backend.endpoint_ids.issubset(profile.endpoint_ids):
            raise ValueError("后端 endpoint_ids 未被 HardwareProfile 原子锁定")
        self.backend = backend
        self.profile = profile
        self.journal = journal
        self._safety_observation = safety_observation

    @property
    def endpoint_ids(self) -> frozenset[str]:
        """返回该模块占用的物理端点集合。"""

        return self.backend.endpoint_ids

    @property
    def has_unsettled_fence(self) -> bool:
        """返回该机械臂是否仍有未物理结算命令。"""

        return bool(self.journal.fenced_command_ids())

    def fenced_command_ids(self) -> tuple[str, ...]:
        """返回私有账本全部未物理结算命令身份。"""

        return self.journal.fenced_command_ids()

    def get_command(self, command_id: str) -> CommandResult | None:
        """只读返回机械臂公共命令投影。"""

        return self.journal.get(command_id)

    def resolve_unknown_as_canceled(
        self,
        command_id: str,
        *,
        witness_id: str,
        reason: str,
        source: str,
    ) -> CommandResult:
        """用操作员物理空闲见证结算 standalone Arm Fence。"""

        return settle_unknown_as_canceled(
            self.journal,
            command_id,
            witness_id=witness_id,
            reason=reason,
            source=source,
        )

    def execute(self, command: RobotCommand) -> CommandResult:
        """幂等执行一个已解析命令，歧义时保留 Fence 且不重放。

        参数：``command`` 不含 Site/Material/地址。返回：权威命令投影。异常：无；确定性错误被转成 rejected/failed。
        """

        if command.hardware_profile_digest != self.profile.digest:
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                "HardwareProfile digest 不匹配",
            )
        if self.journal.get(command.command_id) is None:
            fenced = self.journal.fenced_command_ids()
            if fenced:
                return CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    f"机械臂存在未物理结算命令，禁止新派发: {fenced}",
                )
        try:
            created, existing = self.journal.accept(command)
        except CommandRejectedError as exc:
            return CommandResult(command.command_id, CommandState.REJECTED, str(exc))
        if not created:
            return existing

        rejection = self._pre_dispatch_rejection()
        if rejection:
            return self.journal.update(
                CommandResult(command.command_id, CommandState.REJECTED, rejection)
            )
        self.journal.update(
            CommandResult(command.command_id, CommandState.RUNNING, "机械臂命令已派发")
        )
        try:
            result = self.backend.execute(command)
        except DispatchUnknownError as exc:
            result = CommandResult(
                command.command_id, CommandState.EXECUTION_UNKNOWN, str(exc)
            )
        except CommandRejectedError as exc:
            result = CommandResult(command.command_id, CommandState.FAILED, str(exc))
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"后端异常且派发结果不明: {exc}",
            )
        if result.command_id != command.command_id:
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                "后端返回了不同 command_id",
            )
        interrupted = self.journal.get(command.command_id)
        if (
            interrupted is not None
            and interrupted.state is CommandState.EXECUTION_UNKNOWN
            and self.journal.is_fenced(command.command_id)
        ):
            return interrupted
        return self.journal.update(result)

    def validate_before_dispatch(self, command: RobotCommand) -> None:
        """让 WorkCell 在导轨移动前调用后端的无物理作用预校验。"""

        if command.hardware_profile_digest != self.profile.digest:
            raise CommandRejectedError("HardwareProfile digest 不匹配")
        if self.has_unsettled_fence:
            raise CommandRejectedError("机械臂存在未物理结算 Fence")
        validator = getattr(self.backend, "validate_before_dispatch", None)
        if callable(validator):
            validator(command)

    def request_controlled_stop(
        self, command_id: str, reason: str
    ) -> CommandResult:
        """先锁存执行不确定性，再请求后端普通停止。

        参数：待遏制的命令身份与诊断原因。返回：后端停止请求结果，仅供诊断。
        注意：即使后端确认停止，也不会在此解除本地派发阻断或证明物理结算。
        """

        existing = self.journal.get(command_id)
        if existing is not None and not existing.state.terminal:
            self.journal.update(
                CommandResult(
                    command_id,
                    CommandState.EXECUTION_UNKNOWN,
                    f"已请求机械臂受控停止，等待物理结算: {reason}",
                ),
                fenced=True,
            )
        try:
            result = self.backend.request_stop(command_id, reason)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"机械臂受控停止请求失败: {exc}",
            )
        if result.command_id != command_id:
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "机械臂停止结果 command_id 不匹配",
            )
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        """只对账 UNKNOWN/运行中命令，绝不自动重放物理动作。"""

        existing = self.journal.get(command_id)
        if existing is None:
            raise KeyError(f"命令不存在: {command_id}")
        if existing.state.terminal and not self.journal.is_fenced(command_id):
            return existing
        try:
            result = self.backend.reconcile(command_id)
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, f"对账失败: {exc}"
            )
        if (
            existing.state is CommandState.EXECUTION_UNKNOWN
            and result.state.terminal
        ):
            return CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "已观测到候选终态，必须提供精确物理结算见证才能解除阻断",
                {"candidate_state": result.state.value, "candidate": dict(result.output)},
            )
        return self.journal.update(result)

    def settle_unknown(
        self,
        result: CommandResult,
        evidence: PhysicalSettlementEvidence,
    ) -> CommandResult:
        """用人工或设备精确回执结算 UNKNOWN；不会触发物理重放。"""

        return self.journal.settle(result, evidence)

    def _pre_dispatch_rejection(self) -> str | None:
        """集中执行上线、空闲和硬件许可的新鲜度门禁。"""

        status = self.backend.status()
        if not status.online:
            return "机械臂后端不在线"
        if not status.idle:
            return "机械臂后端非空闲"
        safety = self._safety_observation()
        if not safety.is_fresh():
            return "硬件安全许可未知或过期"
        if not safety.granted or not safety.arm_motion_permitted:
            return "硬件安全链未许可机械臂运动"
        if not safety.concurrent_motion_blocked:
            return "硬件安全链不能证明导轨与机械臂互斥"
        if (
            self.profile.mode is DeploymentMode.PRODUCTION
            and not safety.hardware_enforced
        ):
            return "production profile 缺少经验证硬件互锁"
        return None
