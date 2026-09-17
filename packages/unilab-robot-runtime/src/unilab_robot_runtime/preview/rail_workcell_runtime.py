"""Preview 导轨＋机械臂 WorkCell；无 PLC 互锁，仿真导轨先到位再动臂。"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Any

from unilab_robot_contracts import (
    CommandResult,
    CommandState,
    ObservationState,
    RailMoveCommand,
    RobotCommand,
)

from ..access_motion_backend import AccessMotionBackend


class PreviewRailWorkCellRuntime:
    """RailMountedArm Preview 运行时；接口对齐 RuntimeBinding 期望。"""

    def __init__(
        self,
        *,
        arm_runtime: AccessMotionBackend,
        rail_port: Any,
        settle_timeout_s: float = 30.0,
    ) -> None:
        self._arm = arm_runtime
        self._rail = rail_port
        self._settle_timeout_s = settle_timeout_s
        self.has_unsettled_fence = False

    def execute(self, command: RobotCommand, *, rail_target_ref: str) -> CommandResult:
        rail_child = f"{command.command_id}:rail"
        try:
            self._arm.validate_before_dispatch(command)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                f"Preview WorkCell 派发前校验拒绝: {exc}",
            )

        try:
            self._rail.move(rail_child, rail_target_ref)
            self._wait_rail_settled(rail_child, rail_target_ref)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command.command_id,
                CommandState.FAILED,
                f"Preview 导轨阶段失败: {exc}",
            )

        private = replace(command, command_id=f"{command.command_id}:arm")
        arm_result = self._arm.execute(private)
        if arm_result.state is not CommandState.SUCCEEDED:
            return CommandResult(
                command.command_id,
                arm_result.state,
                arm_result.message,
                {
                    **dict(arm_result.output),
                    "rail_target_ref": rail_target_ref,
                },
            )
        return CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "RAIL_SETTLED → PREVIEW_ARM_COMPLETED",
            {
                **dict(arm_result.output),
                "rail_target_ref": rail_target_ref,
            },
        )

    def move_rail(self, command: RailMoveCommand) -> CommandResult:
        try:
            self._rail.move(command.command_id, command.target_ref)
            self._wait_rail_settled(command.command_id, command.target_ref)
        except Exception as exc:  # noqa: BLE001
            return CommandResult(
                command.command_id,
                CommandState.FAILED,
                f"Preview rail-only 失败: {exc}",
            )
        return CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "Preview rail move completed",
            {"target_ref": command.target_ref},
        )

    def request_controlled_stop(self, command_id: str, reason: str) -> CommandResult:
        return self._arm.request_stop(command_id, reason)

    def close(self) -> None:
        return None

    def _wait_rail_settled(self, command_id: str, target_ref: str) -> None:
        deadline = time.monotonic() + self._settle_timeout_s
        while time.monotonic() < deadline:
            observation = self._rail.observe()
            if (
                observation.state is ObservationState.KNOWN
                and observation.moving is False
                and observation.settled is True
                and observation.target_ref == target_ref
                and observation.completed_command_id == command_id
            ):
                return
            time.sleep(0.02)
        raise TimeoutError(
            f"Preview 导轨未在 {self._settle_timeout_s}s 内到位: {target_ref}"
        )
