"""把 PreviewArmDevice 包装成 Robot runtime 可绑定形态。"""

from __future__ import annotations

from dataclasses import dataclass, field

from unilab_robot_contracts import CommandResult, CommandState

from .preview_arm_device import PreviewArmDevice


@dataclass
class PreviewRuntime:
    """最小 preview 运行时；仅支持 moveJ/home，不支持 RobotCommand。"""

    device: PreviewArmDevice
    has_unsettled_fence: bool = False

    def execute(self, command) -> CommandResult:
        """Preview 模式未注入 PreviewSegmentExecutor 时不接受 RobotCommand。"""

        del command
        return CommandResult(
            command_id="preview-unsupported",
            state=CommandState.FAILED,
            success=False,
            message="Preview runtime 未注入 PreviewSegmentExecutor，不支持 RobotCommand 执行",
            output={},
        )

    def request_controlled_stop(self, command_id: str, reason: str) -> CommandResult:
        del reason
        self.device.stop()
        return CommandResult(
            command_id=command_id,
            state=CommandState.SUCCEEDED,
            success=True,
            message="preview stop requested",
            output={"mode": self.device.mode},
        )

    def close(self) -> None:
        return None


@dataclass
class PreviewRuntimeBinding:
    """供 factory 返回的 preview 绑定载荷。"""

    runtime: PreviewRuntime = field(repr=False)
    preview_device: PreviewArmDevice = field(repr=False)

    @property
    def device(self) -> PreviewArmDevice:
        return self.preview_device
