"""PreviewArmDevice + PreviewSegmentExecutor 的 RobotExecutionBackend 适配。"""

from __future__ import annotations

import threading
import time
from typing import Any

from unilab_robot_contracts import (
    BackendStatus,
    CommandResult,
    CommandState,
    EndEffectorObservation,
    MotionSegment,
    ObservationState,
    RobotCommand,
)

from .preview_arm_device import PreviewArmDevice
from .segment_executor import PreviewSegmentExecutor


class PreviewArmExecutionBackend:
    """AccessMotionBackend 在 Preview 模式下的机械臂执行后端。"""

    def __init__(
        self,
        *,
        device: PreviewArmDevice,
        segment_executor: PreviewSegmentExecutor,
        endpoint_ids: frozenset[str],
        default_duration_s: float = 2.0,
    ) -> None:
        self.device = device
        self._segment_executor = segment_executor
        self.endpoint_ids = endpoint_ids
        self._default_duration_s = default_duration_s
        self._stop = threading.Event()
        self._active_command_id: str | None = None
        self._results: dict[str, CommandResult] = {}

    def status(self) -> BackendStatus:
        busy = not self.device._motion.acquire(blocking=False)
        if not busy:
            self.device._motion.release()
        return BackendStatus(
            online=True,
            idle=not busy,
            active_command_id=self._active_command_id,
            diagnostic="preview-arm-backend",
        )

    def execute(self, command: RobotCommand) -> CommandResult:
        self._active_command_id = command.command_id
        self._stop.clear()
        completed: list[dict[str, str]] = []
        try:
            for segment in command.segments:
                duration = self._segment_duration(segment)
                self._segment_executor.execute_arm_move(
                    self.device,
                    segment,
                    stop_event=self._stop,
                    duration_s=duration,
                )
                completed.append(
                    {
                        "segment_id": segment.segment_id,
                        "target_ref": segment.target_ref,
                        "state": CommandState.SUCCEEDED.value,
                    }
                )
        except RuntimeError as exc:
            if "stopped" in str(exc).lower():
                result = CommandResult(
                    command.command_id,
                    CommandState.CANCELED,
                    str(exc),
                    {"phases": completed},
                )
                self._results[command.command_id] = result
                return result
            result = CommandResult(
                command.command_id,
                CommandState.FAILED,
                str(exc),
                {"phases": completed},
            )
            self._results[command.command_id] = result
            return result
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command.command_id,
                CommandState.FAILED,
                f"Preview arm segment failed: {exc}",
                {"phases": completed},
            )
            self._results[command.command_id] = result
            return result
        finally:
            self._active_command_id = None

        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "Preview arm segments completed",
            {"phases": completed},
        )
        self._results[command.command_id] = result
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        return self._results.get(
            command_id,
            CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "Preview arm command missing reconcile evidence",
            ),
        )

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        del reason
        self._stop.set()
        self.device.stop()
        return CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            "Preview arm stop requested; await physical settlement",
        )

    def end_effector_observation(self) -> EndEffectorObservation:
        holding = self.device.grip > 0.001
        return EndEffectorObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "preview-arm-device",
            holding,
            None,
        )

    def validate_before_dispatch(self, command: RobotCommand) -> None:
        del command

    @property
    def has_unsettled_fence(self) -> bool:
        return False

    def fenced_command_ids(self) -> tuple[str, ...]:
        return ()

    @staticmethod
    def _segment_duration(segment: MotionSegment) -> float:
        raw = segment.parameters.get("duration_s")
        if raw is not None:
            return float(raw)
        return 2.0
