"""把 D9-1S 机械臂、夹爪和负载观测收敛到一个执行边界。"""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import replace
from threading import RLock

from unilab_robot_contracts import (
    BackendStatus,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    EndEffectorPort,
    MotionSegment,
    ObservationState,
    RobotCommand,
    RobotExecutionBackend,
    ToolChangerPort,
    ToolContext,
    WorkCellPhaseKind,
)


class AccessMotionBackend:
    """严格顺序执行 Arm→夹爪→观测→Arm，禁止跳过组合阶段。"""

    def __init__(
        self,
        *,
        arm_backend: RobotExecutionBackend,
        end_effector: EndEffectorPort,
        tool_changer: ToolChangerPort,
        expected_tool_context: ToolContext,
    ) -> None:
        """绑定机械臂后端和独立夹爪端口；二者均由部署组合根选择。"""

        self.arm_backend = arm_backend
        self.end_effector = end_effector
        self.tool_changer = tool_changer
        self.expected_tool_context = expected_tool_context
        self.endpoint_ids = arm_backend.endpoint_ids
        self._results: dict[str, CommandResult] = {}
        self._active_arm_command_id: str | None = None
        self._active_lock = RLock()

    def status(self) -> BackendStatus:
        """复用机械臂连接与空闲状态作为组合动作的派发前门禁。"""

        return self.arm_backend.status()

    def execute(self, command: RobotCommand) -> CommandResult:
        """执行一个完整 AccessMotionBlock，并保留部分执行后的不确定性。

        参数：命令的每个段必须显式携带 ``phase_kind``。返回：组合终态。
        异常：首次物理作用前的结构错误为拒绝；之后任何异常均为派发不明。
        """

        phases = self._prevalidate(command)
        completed: list[Mapping[str, str]] = []
        physical_effect = False
        try:
            for segment, phase_kind in phases:
                if phase_kind is WorkCellPhaseKind.ARM_MOVE:
                    child_id = f"{command.command_id}:arm:{segment.segment_id}"
                    with self._active_lock:
                        self._active_arm_command_id = child_id
                    try:
                        result = self.arm_backend.execute(
                            replace(
                                command,
                                command_id=child_id,
                                segments=(segment,),
                            )
                        )
                    finally:
                        with self._active_lock:
                            self._active_arm_command_id = None
                    physical_effect = True
                elif phase_kind is WorkCellPhaseKind.END_EFFECTOR:
                    result = self._execute_end_effector(command, segment.target_ref)
                    physical_effect = True
                elif phase_kind is WorkCellPhaseKind.OBSERVE:
                    result = self._observe_payload(command, segment.parameters)
                else:
                    raise CommandRejectedError(
                        "standalone Arm 的 AccessMotionBlock 不得包含导轨阶段"
                    )
                completed.append(
                    {
                        "segment_id": segment.segment_id,
                        "target_ref": segment.target_ref,
                        "state": result.state.value,
                    }
                )
                if result.state is not CommandState.SUCCEEDED:
                    return self._non_success(
                        command,
                        result,
                        completed=completed,
                        physical_effect=physical_effect,
                    )
        except CommandRejectedError:
            if not physical_effect:
                raise
            raise DispatchUnknownError(
                "AccessMotionBlock 已产生物理作用，后续阶段被拒绝"
            )
        except DispatchUnknownError:
            raise
        except Exception as exc:
            if not physical_effect:
                raise CommandRejectedError(
                    f"AccessMotionBlock 派发前失败: {exc}"
                ) from exc
            raise DispatchUnknownError(
                f"AccessMotionBlock 部分执行后结果不明: {exc}"
            ) from exc
        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "AccessMotionBlock 的机械臂、夹爪与负载观测全部完成",
            {"phases": completed},
        )
        self._results[command.command_id] = result
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        """返回已知组合终态；缺失完整证据时保持 execution_unknown。"""

        return self._results.get(
            command_id,
            CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "组合动作缺少完整机械臂与夹爪完成见证",
            ),
        )

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """请求机械臂受控停止；夹爪没有通用停止证明，因此不解除 Fence。"""

        with self._active_lock:
            active_arm_command_id = self._active_arm_command_id
        self.arm_backend.request_stop(active_arm_command_id or command_id, reason)
        result = CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            "已请求组合动作受控停止，等待物理结算",
        )
        self._results[command_id] = result
        return result

    def end_effector_observation(self):
        """保留原机械臂后端的末端观测兼容表面。"""

        return self.arm_backend.end_effector_observation()

    def _prevalidate(
        self,
        command: RobotCommand,
    ) -> tuple[tuple[MotionSegment, WorkCellPhaseKind], ...]:
        """首次物理作用前校验所有阶段形态与夹爪初态。"""

        result: list[tuple[MotionSegment, WorkCellPhaseKind]] = []
        seen_end_effector = 0
        seen_observation = 0
        for segment in command.segments:
            raw_kind = segment.parameters.get("phase_kind")
            try:
                kind = WorkCellPhaseKind(str(raw_kind))
            except ValueError as exc:
                raise CommandRejectedError(
                    f"AccessMotionBlock 阶段类型无效: {raw_kind}"
                ) from exc
            if kind is WorkCellPhaseKind.RAIL_MOVE:
                raise CommandRejectedError("standalone AccessMotionBlock 禁止导轨阶段")
            if kind is WorkCellPhaseKind.END_EFFECTOR:
                seen_end_effector += 1
                expected = (
                    "end_effector.grip"
                    if command.action.value == "pick"
                    else "end_effector.release"
                )
                if segment.target_ref != expected:
                    raise CommandRejectedError("夹爪阶段与通用动作不匹配")
            if kind is WorkCellPhaseKind.OBSERVE:
                seen_observation += 1
            result.append((segment, kind))
        if seen_end_effector != 1 or seen_observation != 1:
            raise CommandRejectedError("pick/place 必须各含一个夹爪阶段和负载观测阶段")
        observation = self.end_effector.observe()
        attachment = self.tool_changer.observe()
        active_context = self.tool_changer.active_tool_context
        tool_known = (
            attachment.state is ObservationState.KNOWN
            and time.time() - attachment.observed_at <= attachment.max_age_s
            and attachment.locked is True
            and attachment.tool_ref == self.expected_tool_context.context_id
            and attachment.attachment_generation
            == self.expected_tool_context.attachment_generation
            and active_context.digest == self.expected_tool_context.digest
        )
        if not tool_known:
            raise CommandRejectedError(
                "快换工具身份、锁紧、附着代次或 ToolContext 摘要不匹配"
            )
        if (
            observation.state is not ObservationState.KNOWN
            or time.time() - observation.observed_at > observation.max_age_s
            or observation.holding_payload is None
        ):
            raise CommandRejectedError("夹爪初始观测未知或过期")
        expected_holding = command.action.value == "place"
        if observation.holding_payload is not expected_holding:
            raise CommandRejectedError("夹爪初始负载状态与 pick/place 不匹配")
        return tuple(result)

    def _execute_end_effector(
        self,
        command: RobotCommand,
        target_ref: str,
    ) -> CommandResult:
        """用子命令身份执行抓取或释放。"""

        child_id = f"{command.command_id}:end-effector:{target_ref}"
        if target_ref == "end_effector.grip":
            return self.end_effector.grip(
                child_id,
                payload_profile=command.payload_profile,
            )
        if target_ref == "end_effector.release":
            return self.end_effector.release(child_id)
        raise CommandRejectedError(f"未知夹爪动作: {target_ref}")

    def _observe_payload(
        self,
        command: RobotCommand,
        parameters: Mapping[str, object],
    ) -> CommandResult:
        """验证夹爪负载观测与计划声明的出站状态一致。"""

        expected_state = str(parameters.get("payload_state", ""))
        if expected_state not in {"loaded", "empty"}:
            raise CommandRejectedError("负载观测阶段缺少 loaded/empty 目标")
        observation = self.end_effector.observe()
        known = (
            observation.state is ObservationState.KNOWN
            and time.time() - observation.observed_at <= observation.max_age_s
            and observation.holding_payload is (expected_state == "loaded")
        )
        return CommandResult(
            command.command_id,
            CommandState.SUCCEEDED if known else CommandState.EXECUTION_UNKNOWN,
            "夹爪负载观测已确认" if known else "夹爪负载观测未知、过期或不匹配",
        )

    def _non_success(
        self,
        command: RobotCommand,
        result: CommandResult,
        *,
        completed: list[Mapping[str, str]],
        physical_effect: bool,
    ) -> CommandResult:
        """部分执行后不把失败折叠为可重试普通失败。"""

        state = CommandState.EXECUTION_UNKNOWN if physical_effect else result.state
        combined = CommandResult(
            command.command_id,
            state,
            f"AccessMotionBlock 在 {completed[-1]['segment_id']} 停止: {result.message}",
            {"phases": completed},
        )
        self._results[command.command_id] = combined
        return combined


__all__ = ["AccessMotionBackend"]
