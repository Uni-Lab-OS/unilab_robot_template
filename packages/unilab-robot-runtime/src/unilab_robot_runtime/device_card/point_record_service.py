"""PointSet v3 示教落盘（与执行器 backend 无关）。"""

from __future__ import annotations

from pathlib import Path

from unilab_robot_contracts import (
    patch_authored_composite_target_yaml,
    patch_authored_joint_target_yaml,
)

from .observation_adapters import require_observation_ready
from .observation_types import ExecutorObservation
from .types import RobotTeachPointResult


def teach_joint_target_from_observation(
    observation: ExecutorObservation,
    *,
    point_set_path: str | Path,
    target_ref: str,
    joint_count: int,
    confirm: bool,
    expected_revision: str | None = None,
    rail_position_si: float | None = None,
) -> RobotTeachPointResult:
    require_observation_ready(observation, confirm=confirm)
    normalized = str(target_ref).strip()
    if not normalized:
        raise ValueError("示教落盘必须包含稳定 target_ref")
    joints = list(observation.get("joint_positions_si") or [])
    if len(joints) != joint_count:
        raise ValueError("示教关节数量与 Arm 型号不一致")
    path = Path(point_set_path)
    text = path.read_text(encoding="utf-8")
    rail = rail_position_si
    if rail is None:
        rail = observation.get("rail_position_si")
    if rail is not None and not _authored_target_has_rail(text, normalized):
        rail = None
    if rail is not None:
        patched = patch_authored_composite_target_yaml(
            text,
            normalized,
            joints,
            float(rail),
            joint_count=joint_count,
            expected_revision=expected_revision,
        )
    else:
        if expected_revision is not None:
            import yaml

            loaded = yaml.safe_load(text)
            current = str(loaded.get("revision", "")).strip() if isinstance(loaded, dict) else ""
            if current != str(expected_revision).strip():
                raise ValueError(
                    f"PointSet 修订冲突: expected={expected_revision!r}, actual={current!r}"
                )
        patched = patch_authored_joint_target_yaml(
            text,
            normalized,
            joints,
            joint_count=joint_count,
        )
    path.write_text(patched, encoding="utf-8", newline="\n")
    import yaml

    revised = yaml.safe_load(patched)
    revision = str(revised.get("revision", "")) if isinstance(revised, dict) else ""
    return {
        "target_ref": normalized,
        "target_revision": revision,
    }


def _authored_target_has_rail(text: str, target_ref: str) -> bool:
    """仅当作者层复合点已声明 rail 时才走 composite 写回。"""

    import yaml

    document = yaml.safe_load(text)
    if not isinstance(document, dict):
        return False
    parts = str(target_ref).strip().split(".", 1)
    if len(parts) != 2:
        return False
    group_ref, site_ref = parts
    group = document.get(group_ref)
    if not isinstance(group, dict):
        return False
    targets = group.get("targets")
    if not isinstance(targets, dict):
        return False
    target = targets.get(site_ref)
    return isinstance(target, dict) and "rail" in target
