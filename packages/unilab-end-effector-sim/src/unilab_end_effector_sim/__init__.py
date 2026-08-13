"""夹爪和快换工具的确定性仿真 Adapter。"""

from __future__ import annotations

import time
from collections.abc import Mapping

from unilab_robot_contracts import (
    CommandResult,
    CommandState,
    GripperObservation,
    ObservationState,
    ToolAttachmentObservation,
    ToolContext,
    ToolDefinition,
)


class SimulatedToolChanger:
    """工具型号可替换、附着代次单调递增的仿真快换。"""

    def __init__(self, *, tools: Mapping[str, ToolDefinition]) -> None:
        """冻结可用工具目录并从未附着状态启动。"""

        self.tools = dict(tools)
        if not self.tools:
            raise ValueError("仿真快换至少需要一个工具")
        self._tool_ref: str | None = None
        self._generation = 0
        self._results: dict[str, CommandResult] = {}

    def observe(self) -> ToolAttachmentObservation:
        """返回当前工具、锁紧状态和附着代次。"""

        return ToolAttachmentObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "simulation:tool-changer",
            self._tool_ref,
            self._generation,
            self._tool_ref is not None,
        )

    def change_tool(self, command_id: str, *, tool_ref: str) -> CommandResult:
        """仿真解锁/换装/锁紧，并只在工具变化时增加代次。"""

        if command_id in self._results:
            return self._results[command_id]
        if tool_ref not in self.tools:
            result = CommandResult(
                command_id,
                CommandState.REJECTED,
                "快换工具不在已批准目录",
            )
        else:
            if tool_ref != self._tool_ref:
                self._generation += 1
                self._tool_ref = tool_ref
            result = CommandResult(
                command_id,
                CommandState.SUCCEEDED,
                "仿真快换已锁紧",
                {
                    "tool_ref": self._tool_ref,
                    "attachment_generation": self._generation,
                },
            )
        self._results[command_id] = result
        return result

    @property
    def active_tool_context(self) -> ToolContext:
        """返回与当前锁紧代次绑定的 ToolContext。"""

        if self._tool_ref is None:
            raise RuntimeError("快换尚未附着工具")
        return self.tools[self._tool_ref].context(self._generation)


class SimulatedGripper:
    """依赖已锁紧快换工具的开合夹爪仿真。"""

    def __init__(self, *, tool_changer: SimulatedToolChanger) -> None:
        """绑定快换观测并从张开、无负载状态启动。"""

        self.tool_changer = tool_changer
        self._closed = False
        self._holding = False
        self._results: dict[str, CommandResult] = {}

    def observe(self) -> GripperObservation:
        """返回仿真开合和负载状态。"""

        return GripperObservation(
            ObservationState.KNOWN,
            time.time(),
            1.0,
            "simulation:gripper",
            self._closed,
            self._holding,
        )

    def grip(self, command_id: str, *, payload_profile: str) -> CommandResult:
        """已锁紧工具时闭合并确认持有负载。"""

        if command_id in self._results:
            return self._results[command_id]
        attachment = self.tool_changer.observe()
        if not payload_profile.strip() or attachment.locked is not True:
            result = CommandResult(
                command_id,
                CommandState.REJECTED,
                "夹爪抓取缺少负载类型或已锁紧工具",
            )
        else:
            self._closed = True
            self._holding = True
            result = CommandResult(
                command_id,
                CommandState.SUCCEEDED,
                "仿真夹爪已确认抓取",
                {"payload_profile": payload_profile},
            )
        self._results[command_id] = result
        return result

    def release(self, command_id: str) -> CommandResult:
        """张开夹爪并确认不再持有负载。"""

        if command_id in self._results:
            return self._results[command_id]
        self._closed = False
        self._holding = False
        result = CommandResult(
            command_id,
            CommandState.SUCCEEDED,
            "仿真夹爪已确认释放",
        )
        self._results[command_id] = result
        return result


__version__ = "0.1.0"

__all__ = ["SimulatedGripper", "SimulatedToolChanger"]
