"""仅供导轨维护或真实独立部署使用的薄包装。"""

from __future__ import annotations

from unilab_robot_contracts import RailStateObservation

from .rail_module import RailModule


class StandaloneRailDevice:
    """公开单轴 move/status；复合 WorkCell 不得同时激活它。"""

    def __init__(self, module: RailModule) -> None:
        """注入同一个可复用 RailModule。"""

        self._module = module

    def move(self, command_id: str, target_ref: str) -> RailStateObservation:
        """执行独立维护移动；生产调度仍需完整占用与互锁。"""

        return self._module.move_and_settle(command_id, target_ref)

    def status(self) -> RailStateObservation:
        """读取导轨规范状态。"""

        return self._module.observe()
