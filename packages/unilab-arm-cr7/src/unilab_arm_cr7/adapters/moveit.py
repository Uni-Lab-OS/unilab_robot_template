"""仅控制 CR7 六轴、与 RViz 生命周期完全解耦的 MoveIt 后端。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any, Protocol

from unilab_robot_contracts import (
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    ResolvedMotionTarget,
    RobotCommand,
)

from ._support import BackendObservationMixin, validate_completion_receipt

CR7_JOINT_NAMES = (
    "cr7_joint_1",
    "cr7_joint_2",
    "cr7_joint_3",
    "cr7_joint_4",
    "cr7_joint_5",
    "cr7_joint_6",
)


class MoveGroupPort(Protocol):
    """move_group 的无 RViz 执行端口。"""

    def execute_joint_target(
        self,
        *,
        group_name: str,
        joint_names: Sequence[str],
        target: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """规划并执行六轴目标，返回 move_group 完成见证。"""

    def execute_cartesian_target(
        self,
        *,
        group_name: str,
        frame_ref: str,
        xyz_m: Sequence[float],
        orientation_xyzw: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """规划并执行绝对 TCP 目标，返回 move_group 完成见证。"""

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """查询 move_group 执行见证。"""

    def cancel(self, command_id: str) -> bool:
        """请求取消并返回是否已确认停止。"""

class MoveItBackend(BackendObservationMixin):
    """headless MoveIt 后端；构造和执行均不 import、启动或探测 RViz。"""

    def __init__(
        self,
        *,
        port: MoveGroupPort,
        endpoint_ids: frozenset[str],
        targets: Mapping[str, ResolvedMotionTarget],
        group_name: str = "cr7_arm",
    ) -> None:
        """注入 move_group port 与派发前已解析的运动目标。

        参数：关节目标必须正好六轴；笛卡尔目标必须是绝对位姿。
        返回：无。异常：原始数组、七轴或引用漂移时立即拒绝启动。
        """

        normalized: dict[str, ResolvedMotionTarget] = {}
        for name, target in targets.items():
            if name != target.target_ref:
                raise ValueError(f"MoveIt target_ref 索引漂移: {name}")
            if isinstance(target, ResolvedJointTarget):
                if len(target.joint_positions) != len(CR7_JOINT_NAMES):
                    raise ValueError(f"MoveIt CR7 target 必须为六轴: {name}")
            elif not isinstance(target, ResolvedCartesianTarget):
                raise ValueError(f"MoveIt target 必须由 MotionTargetResolver 解析: {name}")
            normalized[name] = target
        self.port = port
        self.endpoint_ids = endpoint_ids
        self.targets = normalized
        self.group_name = group_name
        self._results: dict[str, CommandResult] = {}
        self._initialize_observation()

    @staticmethod
    def required_launch_components() -> tuple[str, ...]:
        """返回执行所需组件；故意不包含 RViz。"""

        return (
            "robot_state_publisher",
            "controller_manager",
            "joint_state_broadcaster",
            "move_group",
        )

    def execute(self, command: RobotCommand) -> CommandResult:
        """按顺序执行六轴目标；不存在 target_ref 时派发前拒绝。"""

        missing = [
            segment.target_ref
            for segment in command.segments
            if segment.target_ref not in self.targets
        ]
        if missing:
            raise CommandRejectedError(f"MoveIt point-set 缺少 target_ref: {missing}")
        self._active_command_id = command.command_id
        outputs: list[Mapping[str, Any]] = []
        try:
            for segment in command.segments:
                target = self.targets[segment.target_ref]
                outputs.append(
                    validate_completion_receipt(
                        self._execute_target(
                            target,
                            command_id=command.command_id,
                            parameters=segment.parameters,
                        ),
                        command_id=command.command_id,
                        source="MoveIt",
                    )
                )
        except Exception as exc:
            raise DispatchUnknownError(f"MoveIt 执行结果不明: {exc}") from exc
        finally:
            self._active_command_id = None
        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "move_group 返回完成见证",
            {"segments": outputs},
        )
        self._results[command.command_id] = result
        return result

    def _execute_target(
        self,
        target: ResolvedMotionTarget,
        *,
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """按解析后的目标类型选择唯一 MoveIt 执行入口。

        参数：``target`` 已由共享解析器冻结；其余参数绑定当前命令段。
        返回：精确绑定 ``command_id`` 的完成见证。
        """

        if isinstance(target, ResolvedJointTarget):
            return self.port.execute_joint_target(
                group_name=self.group_name,
                joint_names=CR7_JOINT_NAMES,
                target=target.joint_positions,
                command_id=command_id,
                parameters=parameters,
            )
        return self.port.execute_cartesian_target(
            group_name=self.group_name,
            frame_ref=target.pose.frame_ref,
            xyz_m=target.pose.xyz_m,
            orientation_xyzw=target.pose.orientation_xyzw,
            command_id=command_id,
            parameters=parameters,
        )

    def reconcile(self, command_id: str) -> CommandResult:
        """从 move_group 执行记录对账，不依赖 RViz。"""

        observed = self.port.query_command(command_id)
        if observed is None:
            return CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, "move_group 无完成见证"
            )
        state = CommandState(
            str(observed.get("state", CommandState.EXECUTION_UNKNOWN.value))
        )
        if state.terminal:
            try:
                validate_completion_receipt(
                    observed, command_id=command_id, source="MoveIt"
                )
            except ValueError as exc:
                return CommandResult(
                    command_id, CommandState.EXECUTION_UNKNOWN, str(exc), observed
                )
        return CommandResult(
            command_id, state, str(observed.get("message", "MoveIt 对账")), observed
        )

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """取消仅在 move_group 明确确认停止时结算。"""

        confirmed = self.port.cancel(command_id)
        state = CommandState.CANCELED if confirmed else CommandState.EXECUTION_UNKNOWN
        return CommandResult(command_id, state, f"MoveIt cancel: {reason}")
