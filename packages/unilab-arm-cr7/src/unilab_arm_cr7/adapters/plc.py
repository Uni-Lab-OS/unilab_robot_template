"""配置驱动的 PLC 机械臂适配器；站点地址只由部署配置提供。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

from unilab_robot_contracts import (
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    RobotCommand,
)

from ._support import BackendObservationMixin


class PLCVariablePort(Protocol):
    """可由真实 PLC gateway 或 PLC-Sim 实现的变量端口。"""

    def read(self, variable: str) -> Any:
        """读取一个部署配置中的原始变量。"""

    def write(self, variable: str, value: Any) -> None:
        """写入一个部署配置中的原始变量。"""

    def wait_equal(self, variable: str, expected: Any, timeout_s: float) -> bool:
        """等待变量达到确定值；超时返回 False。"""


@dataclass(frozen=True)
class PLCProgramBinding:
    """一个 target_ref 到经验证 PLC 程序和握手节点的部署绑定。"""

    program_number: int
    command_variable: str
    write_done_variable: str
    completion_variable: str
    home_variable: str
    write_allowed_variable: str
    parameter_variables: Mapping[str, str] = field(default_factory=dict)
    timeout_s: float = 300.0


class PLCBackend(BackendObservationMixin):
    """通过普通 PLC 程序握手执行 RobotCommand；安全互锁仍由硬件链负责。"""

    def __init__(
        self,
        *,
        port: PLCVariablePort,
        endpoint_ids: frozenset[str],
        programs: Mapping[str, PLCProgramBinding],
    ) -> None:
        """注入 PLC 变量端口、物理端点和 exact program-set。"""

        self.port = port
        self.endpoint_ids = endpoint_ids
        self.programs = dict(programs)
        self._results: dict[str, CommandResult] = {}
        self._initialize_observation()

    def execute(self, command: RobotCommand) -> CommandResult:
        """依次执行命令段；任何派发后通信歧义都进入 execution_unknown。"""

        if self._active_command_id is not None:
            raise CommandRejectedError("PLC 机械臂后端已有活动命令")
        bindings = self._prevalidate(command)
        self._active_command_id = command.command_id
        dispatched_segments = 0
        try:
            for segment, binding in zip(command.segments, bindings, strict=True):
                self._execute_segment(binding, segment.parameters)
                dispatched_segments += 1
            result = CommandResult(
                command.command_id, CommandState.SUCCEEDED, "PLC 完成码与 Home 已见证"
            )
            self._results[command.command_id] = result
            return result
        except CommandRejectedError as exc:
            if dispatched_segments == 0:
                raise
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"PLC 部分段已执行，后续段拒绝时物理结果不明: {exc}",
            )
            self._results[command.command_id] = result
            raise DispatchUnknownError(result.message) from exc
        except Exception as exc:
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"PLC 派发结果不明: {exc}",
            )
            self._results[command.command_id] = result
            raise DispatchUnknownError(result.message) from exc
        finally:
            self._active_command_id = None

    def reconcile(self, command_id: str) -> CommandResult:
        """读取已知结果；未知命令不进行自动重发。"""

        return self._results.get(
            command_id,
            CommandResult(
                command_id, CommandState.EXECUTION_UNKNOWN, "PLC 无法证明命令物理结果"
            ),
        )

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """普通 PLC 合同没有可证明停止的统一接口，因此保持 execution_unknown。"""

        result = CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            f"PLC 停止结果无法确认: {reason}",
        )
        self._results[command_id] = result
        return result

    def _prevalidate(self, command: RobotCommand) -> tuple[PLCProgramBinding, ...]:
        """首次物理写入前校验全部段和参数，避免结构错误造成部分执行。"""

        bindings: list[PLCProgramBinding] = []
        for segment in command.segments:
            binding = self.programs.get(segment.target_ref)
            if binding is None:
                raise CommandRejectedError(
                    f"PLC program-set 不包含 target_ref: {segment.target_ref}"
                )
            unknown = set(segment.parameters).difference(binding.parameter_variables)
            if unknown:
                raise CommandRejectedError(
                    f"PLC 程序不接受参数: {sorted(unknown)}"
                )
            bindings.append(binding)
        return tuple(bindings)

    def _execute_segment(
        self, binding: PLCProgramBinding, parameters: Mapping[str, Any]
    ) -> None:
        """执行一个已解析 program binding，并验证完成后 Home 见证。"""

        if not bool(self.port.read(binding.write_allowed_variable)):
            raise CommandRejectedError("PLC 未允许写入机器人任务")
        if int(self.port.read(binding.completion_variable) or 0) != 0:
            raise CommandRejectedError(
                "PLC 上一轮完成码尚未清零，拒绝把旧见证当作新完成"
            )
        for name, value in parameters.items():
            variable = binding.parameter_variables[name]
            self.port.write(variable, value)
        self.port.write(binding.command_variable, binding.program_number)
        self.port.write(binding.write_done_variable, True)
        if not self.port.wait_equal(
            binding.completion_variable, binding.program_number, binding.timeout_s
        ):
            raise TimeoutError("等待 PLC 机器人完成码超时")
        if not self.port.wait_equal(binding.home_variable, True, binding.timeout_s):
            raise TimeoutError("PLC 完成后 Robot_Home 未建立")
        self.port.write(binding.write_done_variable, False)
