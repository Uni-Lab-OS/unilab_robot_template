"""机械臂卡片运行时上下文（领域注入）。"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import Any

from .types import RobotDebugPointTarget, RobotDebugRail, RobotRailMotionResult


CatalogProvider = Callable[[object], Iterable[Any]]
TargetProjector = Callable[[Any], RobotDebugPointTarget]
RailProvider = Callable[[object, object], tuple[RobotDebugRail | None, object | None]]
VisionProvider = Callable[[], dict[str, object]]
MotionProfileProvider = Callable[[object], str]
RailMover = Callable[[object, float], RobotRailMotionResult]
RecordVisionHook = Callable[[str, str | None], None]


@dataclass(slots=True)
class ArmCardContext:
    """把领域差异收敛为可注入钩子，供 template 卡片深模块复用。"""

    boot_id_prefix: str
    catalog_entries: CatalogProvider | None = None
    project_target: TargetProjector | None = None
    rail_snapshot: RailProvider | None = None
    move_rail: RailMover | None = None
    vision_snapshot: VisionProvider | None = None
    motion_profile_ref: MotionProfileProvider | None = None
    on_record_with_vision: RecordVisionHook | None = None
    max_joint_jog_deg: float | None = None
    include_vision_in_snapshot: bool = False

    def resolve_catalog(self, port: object) -> list[Any]:
        if self.catalog_entries is not None:
            return list(self.catalog_entries(port))
        catalog = getattr(port, "commissioning_target_catalog", None)
        if catalog is None:
            return []
        return list(catalog)

    def default_motion_profile_ref(self, port: object) -> str:
        if self.motion_profile_ref is not None:
            return self.motion_profile_ref(port)
        return str(getattr(port, "commissioning_motion_profile_ref", "") or "").strip()
