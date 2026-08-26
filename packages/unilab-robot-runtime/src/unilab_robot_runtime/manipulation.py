"""维护/仿真中的机械臂、快换与夹爪组合序列。"""

from __future__ import annotations

import time
from dataclasses import dataclass

from unilab_robot_contracts import (
    CommandResult,
    CommandState,
    EndEffectorPort,
    MoveTargetCommand,
    RobotCommissioningPort,
    ToolChangerPort,
    ToolContextActivator,
)


@dataclass(frozen=True, slots=True)
class ManipulationSequence:
    """一个不把坐标写进 Workflow 的版本化抓放运动序列。"""

    sequence_id: str
    point_set_revision: str
    ready_target_ref: str
    approach_target_ref: str
    interaction_target_ref: str
    retract_target_ref: str
    tool_ref: str
    payload_profile: str

    def __post_init__(self) -> None:
        """要求所有身份和目标引用非空。"""

        if any(
            not value.strip()
            for value in (
                self.sequence_id,
                self.point_set_revision,
                self.ready_target_ref,
                self.approach_target_ref,
                self.interaction_target_ref,
                self.retract_target_ref,
                self.tool_ref,
                self.payload_profile,
            )
        ):
            raise ValueError("ManipulationSequence 字段不得为空")


class ManipulationSequenceRunner:
    """执行 tool-change→arm→gripper→arm 的封闭维护/仿真序列。"""

    def __init__(
        self,
        *,
        commissioning: RobotCommissioningPort,
        end_effector: EndEffectorPort,
        tool_changer: ToolChangerPort,
        tool_context_activator: ToolContextActivator,
        hardware_profile_digest: str,
        source_boot_id: str,
        motion_profile_ref: str = "maintenance-slow",
        velocity_scale: float = 0.05,
        acceleration_scale: float = 0.05,
    ) -> None:
        """注入四个公共端口，并固定低速维护 profile。"""

        if not hardware_profile_digest.strip() or not source_boot_id.strip():
            raise ValueError("组合序列必须绑定 HardwareProfile 与 boot_id")
        if velocity_scale > 0.1 or acceleration_scale > 0.1:
            raise ValueError("组合调试速度和加速度不得超过 10%")
        self.commissioning = commissioning
        self.end_effector = end_effector
        self.tool_changer = tool_changer
        self.tool_context_activator = tool_context_activator
        self.hardware_profile_digest = hardware_profile_digest
        self.source_boot_id = source_boot_id
        self.motion_profile_ref = motion_profile_ref
        self.velocity_scale = velocity_scale
        self.acceleration_scale = acceleration_scale
        self._sequence = 0

    def pick(self, sequence: ManipulationSequence) -> CommandResult:
        """执行 ready→approach→interaction→抓取→retract→ready。"""

        steps: list[dict[str, str]] = []
        tool_result = self.tool_changer.change_tool(
            f"{sequence.sequence_id}:tool",
            tool_ref=sequence.tool_ref,
        )
        steps.append(_step("tool_change", tool_result))
        if not tool_result.success:
            return _sequence_failure(sequence.sequence_id, tool_result, steps)
        attachment = self.tool_changer.observe()
        attachment_age = time.time() - attachment.observed_at
        if (
            attachment.tool_ref != sequence.tool_ref
            or attachment.locked is not True
            or not 0.0 <= attachment_age <= attachment.max_age_s
        ):
            return CommandResult(
                sequence.sequence_id,
                CommandState.EXECUTION_UNKNOWN,
                "快换完成后工具身份、锁紧或新鲜度未确认",
                {"steps": steps},
            )
        self.tool_context_activator.activate_tool_context(
            self.tool_changer.active_tool_context
        )
        for name, target_ref in (
            ("ready", sequence.ready_target_ref),
            ("approach", sequence.approach_target_ref),
            ("interaction", sequence.interaction_target_ref),
        ):
            result = self._move(sequence, name, target_ref)
            steps.append(_step(name, result))
            if not result.success:
                return _sequence_failure(sequence.sequence_id, result, steps)
        grip = self.end_effector.grip(
            f"{sequence.sequence_id}:grip",
            payload_profile=sequence.payload_profile,
        )
        steps.append(_step("grip", grip))
        if not grip.success:
            return _sequence_failure(sequence.sequence_id, grip, steps)
        observation = self.end_effector.observe()
        observation_age = time.time() - observation.observed_at
        if (
            observation.holding_payload is not True
            or not 0.0 <= observation_age <= observation.max_age_s
        ):
            return CommandResult(
                sequence.sequence_id,
                CommandState.EXECUTION_UNKNOWN,
                "夹爪命令完成但负载观测未确认",
                {"steps": steps},
            )
        for name, target_ref in (
            ("retract", sequence.retract_target_ref),
            ("return_ready", sequence.ready_target_ref),
        ):
            result = self._move(sequence, name, target_ref)
            steps.append(_step(name, result))
            if not result.success:
                return _sequence_failure(sequence.sequence_id, result, steps)
        return CommandResult(
            sequence.sequence_id,
            CommandState.SUCCEEDED,
            "机械臂抓取组合序列完成",
            {"steps": steps},
        )

    def place(self, sequence: ManipulationSequence) -> CommandResult:
        """执行同一运动序列，并在 interaction 处释放负载。"""

        steps: list[dict[str, str]] = []
        attachment = self.tool_changer.observe()
        if attachment.tool_ref != sequence.tool_ref or attachment.locked is not True:
            return CommandResult(
                sequence.sequence_id,
                CommandState.REJECTED,
                "place 所需工具未锁紧",
            )
        self.tool_context_activator.activate_tool_context(
            self.tool_changer.active_tool_context
        )
        for name, target_ref in (
            ("ready", sequence.ready_target_ref),
            ("approach", sequence.approach_target_ref),
            ("interaction", sequence.interaction_target_ref),
        ):
            result = self._move(sequence, name, target_ref)
            steps.append(_step(name, result))
            if not result.success:
                return _sequence_failure(sequence.sequence_id, result, steps)
        release = self.end_effector.release(f"{sequence.sequence_id}:release")
        steps.append(_step("release", release))
        if not release.success:
            return _sequence_failure(sequence.sequence_id, release, steps)
        for name, target_ref in (
            ("retract", sequence.retract_target_ref),
            ("return_ready", sequence.ready_target_ref),
        ):
            result = self._move(sequence, name, target_ref)
            steps.append(_step(name, result))
            if not result.success:
                return _sequence_failure(sequence.sequence_id, result, steps)
        return CommandResult(
            sequence.sequence_id,
            CommandState.SUCCEEDED,
            "机械臂放置组合序列完成",
            {"steps": steps},
        )

    def _move(
        self,
        sequence: ManipulationSequence,
        step_name: str,
        target_ref: str,
    ) -> CommandResult:
        """生成并执行一个绑定相同点位版本的低速目标命令。"""

        self._sequence += 1
        return self.commissioning.execute_commissioning(
            MoveTargetCommand(
                command_id=f"{sequence.sequence_id}:{step_name}",
                hardware_profile_digest=self.hardware_profile_digest,
                source_boot_id=self.source_boot_id,
                monotonic_sequence=self._sequence,
                motion_profile_ref=self.motion_profile_ref,
                velocity_scale=self.velocity_scale,
                acceleration_scale=self.acceleration_scale,
                target_ref=target_ref,
                target_revision=sequence.point_set_revision,
            )
        )


def _step(name: str, result: CommandResult) -> dict[str, str]:
    """把子步骤结果投影为稳定审计记录。"""

    return {
        "name": name,
        "command_id": result.command_id,
        "state": result.state.value,
    }


def _sequence_failure(
    sequence_id: str,
    result: CommandResult,
    steps: list[dict[str, str]],
) -> CommandResult:
    """保留子命令 UNKNOWN/失败语义，不将其折叠为普通失败。"""

    return CommandResult(
        sequence_id,
        result.state,
        f"组合序列在 {result.command_id} 停止: {result.message}",
        {"steps": steps},
    )


__all__ = ["ManipulationSequence", "ManipulationSequenceRunner"]
