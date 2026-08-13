"""PointSet v3 的身份、摘要和基础字段校验。"""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from typing import Any

from .grid import RailTargetModel
from .point_set_types import AccessMotionBlock, InstallationCalibration
from .targets import (
    ArmTargetModel,
    ResolvedMotionTarget,
    ToolContext,
    numeric_sequence,
)

DIGEST_PATTERN = re.compile(r"^[0-9a-f]{64}$")
DEVICE_REF_PATTERN = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
RESERVED_POINT_SET_KEYS = frozenset(
    {
        "schema",
        "revision",
        "description",
        "source",
        "qualification",
        "notes",
        "components",
        "installation_calibration",
        "global",
    }
)


def validate_components(
    value: Any,
    *,
    arm_model: ArmTargetModel,
    tool_context: ToolContext,
    rail_model: RailTargetModel | None,
) -> None:
    """验证 PointSet 组件只引用精确装配的型号和 ToolContext。"""

    components = mapping(value, "components")
    if set(components).difference({"arm", "rail"}):
        raise ValueError("PointSet v3 components 只允许 arm 和 rail")
    arm = mapping(components.get("arm"), "components.arm")
    if str(arm.get("model_ref", "")) != arm_model.model_ref:
        raise ValueError("components.arm.model_ref 与精确 Arm 型号不一致")
    if str(arm.get("tool_context_ref", "")) != tool_context.context_id:
        raise ValueError("components.arm.tool_context_ref 与活动 ToolContext 不一致")
    rail_value = components.get("rail")
    if rail_model is None and rail_value is not None:
        raise ValueError("单机械臂 PointSet 不得声明 rail component")
    if rail_model is not None:
        rail = mapping(rail_value, "components.rail")
        if str(rail.get("model_ref", "")) != rail_model.model_ref:
            raise ValueError("components.rail.model_ref 与精确 Rail 型号不一致")


def validate_calibration_ref(
    value: Any,
    calibration: InstallationCalibration,
) -> None:
    """要求 PointSet 精确锁定安装标定 revision 和 digest。"""

    reference = mapping(value, "installation_calibration")
    if str(reference.get("revision", "")) != calibration.revision:
        raise ValueError("PointSet installation calibration revision 不匹配")
    if str(reference.get("digest", "")).lower() != calibration.digest.lower():
        raise ValueError("PointSet installation calibration digest 不匹配")


def arm_digest_payload(
    arm_target_ref: str | None,
    targets: Mapping[str, ResolvedMotionTarget],
    access: AccessMotionBlock | None,
) -> Any:
    """把一个点位包依赖的机械臂目标转换为稳定摘要载荷。"""

    if arm_target_ref is None:
        return None
    refs = [arm_target_ref]
    if access is not None:
        refs.extend(
            [
                access.entry_target_ref,
                access.approach_target_ref,
                *access.transit_in,
                *access.transit_out,
            ]
        )
    return {target_ref: repr(targets[target_ref]) for target_ref in sorted(set(refs))}


def source_digest(point_set: Mapping[str, Any], supplied: str | None) -> str:
    """验证发布原始字节摘要，测试未提供时使用规范 JSON 摘要。"""

    if supplied is not None:
        normalized = supplied.lower()
        if DIGEST_PATTERN.fullmatch(normalized) is None:
            raise ValueError("source_digest 必须是 64 位 SHA-256")
        return normalized
    return stable_digest(point_set)


def stable_digest(value: Any) -> str:
    """计算配置或解析结果的稳定 SHA-256 摘要。"""

    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def target_mapping(value: Any, field: str) -> Mapping[str, Any]:
    """读取机械臂目标，并禁止 target-level unit。"""

    raw = mapping(value, field)
    if "unit" in raw or "joint_names" in raw:
        raise ValueError(f"{field} 禁止重复 unit 或 joint_names")
    return raw


def vector3(value: Any, field: str) -> tuple[float, float, float]:
    """解析接近运动块的三维米制偏移。"""

    values = numeric_sequence(value, field)
    if len(values) != 3:
        raise ValueError(f"{field} 必须包含三个有限数")
    return values  # type: ignore[return-value]


def string_tuple(value: Any, field: str) -> tuple[str, ...]:
    """解析有序且不重复的稳定目标引用列表。"""

    if isinstance(value, (str, bytes)):
        raise TypeError(f"{field} 必须是目标引用列表")
    try:
        values = tuple(str(item).strip() for item in value)
    except TypeError as exc:
        raise ValueError(f"{field} 必须是目标引用列表") from exc
    if any(not item for item in values) or len(set(values)) != len(values):
        raise ValueError(f"{field} 不得包含空值或重复引用")
    return values


def mapping(value: Any, field: str) -> Mapping[str, Any]:
    """要求 PointSet 字段是对象。"""

    if not isinstance(value, Mapping):
        raise TypeError(f"{field} 必须是对象")
    return value
