"""厂商 TCP/SDK 执行适配器。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol

from unilab_robot_contracts import (
    CommandResult,
    CommandState,
    DispatchUnknownError,
    RobotCommand,
)

from ._support import BackendObservationMixin, validate_completion_receipt


class RobotSDKPort(Protocol):
    """厂家 SDK 的受限高层目标端口。"""

    def execute_target(
        self, target_ref: str, parameters: Mapping[str, Any], command_id: str
    ) -> Mapping[str, Any]:
        """执行经部署白名单解析的目标；不得接收任意脚本。"""

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """查询命令见证；找不到时返回 ``None``。"""

    def request_stop(self, command_id: str, reason: str) -> bool:
        """请求普通停止并返回是否已确认。"""

class TcpSdkBackend(BackendObservationMixin):
    """把厂家 TCP/SDK 收敛到 RobotExecutionBackend。"""

    def __init__(self, *, port: RobotSDKPort, endpoint_ids: frozenset[str]) -> None:
        """注入受限 SDK port 和物理端点。"""

        self.port = port
        self.endpoint_ids = endpoint_ids
        self._results: dict[str, CommandResult] = {}
        self._initialize_observation()

    def execute(self, command: RobotCommand) -> CommandResult:
        """顺序执行内部段；通信异常按派发歧义处理。"""

        self._active_command_id = command.command_id
        outputs: list[Mapping[str, Any]] = []
        try:
            for segment in command.segments:
                outputs.append(
                    validate_completion_receipt(
                        self.port.execute_target(
                            segment.target_ref,
                            segment.parameters,
                            command.command_id,
                        ),
                        command_id=command.command_id,
                        source="TCP/SDK",
                    )
                )
        except Exception as exc:
            raise DispatchUnknownError(f"TCP/SDK 派发结果不明: {exc}") from exc
        finally:
            self._active_command_id = None
        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "SDK 返回完成见证",
            {"segments": outputs},
        )
        self._results[command.command_id] = result
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        """通过厂家命令查询接口对账，不自动重发。"""

        observed = self.port.query_command(command_id)
        if observed is None:
            return CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, "SDK 无命令见证"
            )
        state = CommandState(
            str(observed.get("state", CommandState.EXECUTION_UNKNOWN.value))
        )
        if state.terminal:
            try:
                validate_completion_receipt(
                    observed, command_id=command_id, source="TCP/SDK"
                )
            except ValueError as exc:
                return CommandResult(
                    command_id, CommandState.EXECUTION_UNKNOWN, str(exc), observed
                )
        result = CommandResult(
            command_id, state, str(observed.get("message", "SDK 对账")), observed
        )
        self._results[command_id] = result
        return result

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """只有厂家明确确认停止时才结算 canceled。"""

        confirmed = self.port.request_stop(command_id, reason)
        state = CommandState.CANCELED if confirmed else CommandState.EXECUTION_UNKNOWN
        result = CommandResult(
            command_id, state, "SDK 已确认停止" if confirmed else "SDK 停止结果不明"
        )
        self._results[command_id] = result
        return result
