"""单机械臂部署的可选公共包装；复合 WorkCell 不得实例化它。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from unilab_robot_contracts import CommandResult, RobotCommand

from .arm_module import ArmModule


class StandaloneArmDevice:
    """只为真实 standalone 部署提供一个公共 Device 语义表面。"""

    def __init__(
        self, module: ArmModule, resolve_action: Callable[..., RobotCommand]
    ) -> None:
        """注入同一 ArmModule 与部署动作解析器，不复制控制实现。"""

        self._module = module
        self._resolve_action = resolve_action

    def pick(self, **arguments: Any) -> CommandResult:
        """把通用 pick 解析成 RobotCommand 后交给 ArmModule。"""

        return self._module.execute(self._resolve_action("pick", **arguments))

    def place(self, **arguments: Any) -> CommandResult:
        """把通用 place 解析成 RobotCommand 后交给 ArmModule。"""

        return self._module.execute(self._resolve_action("place", **arguments))

    def pour(self, **arguments: Any) -> CommandResult:
        """把通用 pour 解析成内部运动段后交给 ArmModule。"""

        return self._module.execute(self._resolve_action("pour", **arguments))
