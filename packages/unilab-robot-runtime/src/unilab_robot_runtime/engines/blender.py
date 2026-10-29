"""OS Blender ``Scene`` 上的机械臂引擎；bpy/OS 只在实例化时惰性导入。"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .kinematic import KinematicArmEngine


class BlenderArmEngine(KinematicArmEngine):
    """复用 kinematic 接缝，在 OS ``blender_kinematic.Scene`` 上读写关节标记。

    模块顶层不导入 bpy 或 OS；只有在 Blender 进程内实例化时才加载
    ``unilabos.simulation.blender_kinematic``。Blender 之外的 fake 测试使用
    ``KinematicArmEngine``。场景对象必须已经由调用方创建。
    """

    def __init__(self, scene: Any, joint_objects: Mapping[str, str]) -> None:
        try:
            from unilabos.simulation.blender_kinematic import Scene
        except ImportError as error:  # pragma: no cover - 依赖缺失路径
            raise RuntimeError(
                "BlenderArmEngine 需要 Uni-Lab-OS 的 blender_kinematic.Scene，"
                "且只能在 Blender 进程内使用"
            ) from error
        if not isinstance(scene, Scene):
            raise TypeError("BlenderArmEngine.scene 必须是 OS blender_kinematic.Scene")
        super().__init__(scene, joint_objects)


__all__ = ["BlenderArmEngine"]
