"""领域机械臂关节命名：canonical 用 joint_N，qualified 只绑 device_id。"""

from __future__ import annotations

import re

# canonical 只允许 joint_<正整数>，禁止型号 slug、禁止 device_id
_CANONICAL_JOINT = re.compile(r"^joint_[1-9]\d*$")

# qualified 合法形态：{device}_joint_N；多一段 slug 即非法（与具体型号无关）
def legacy_qualified_joint_pattern(device_id: str) -> re.Pattern[str]:
    escaped = re.escape(device_id)
    # 匹配 {device}_{slug}_joint_（含 f-string 模板，末尾可无数字）
    return re.compile(rf"{escaped}_(?!joint_\d)[a-z0-9_]+_joint_")


def expected_canonical_joint_names(*, dof: int = 6) -> tuple[str, ...]:
    """设备无关 canonical：joint_1 .. joint_N。"""

    if dof < 1:
        raise ValueError("dof 必须 >= 1")
    return tuple(f"joint_{index}" for index in range(1, dof + 1))


def expected_qualified_joint_names(*, device_id: str, dof: int = 6) -> tuple[str, ...]:
    """MoveIt / ros2_control / JointStateNameMap 的完全限定名。"""

    return tuple(
        f"{device_id}_{name}" for name in expected_canonical_joint_names(dof=dof)
    )


def is_valid_canonical_joint_name(name: str) -> bool:
    return _CANONICAL_JOINT.fullmatch(str(name).strip()) is not None


def validate_canonical_joint_names(names: tuple[str, ...], *, dof: int = 6) -> list[str]:
    """校验 canonical 层：必须是 joint_1..joint_dof，不得含型号或 device_id。"""

    errors: list[str] = []
    expected = expected_canonical_joint_names(dof=dof)
    if len(names) != dof:
        errors.append(f"kinematic_joints 必须是 {dof} 轴，当前 {len(names)} 个")
    if names != expected:
        errors.append(
            "kinematic_joints 必须是 "
            f"{expected[0]}..{expected[-1]}，当前 {names}"
        )
    for name in names:
        if not is_valid_canonical_joint_name(name):
            errors.append(
                f"canonical 关节名必须符合 joint_<N>，不得含型号 slug 或 device_id: {name!r}"
            )
    return errors


def find_legacy_qualified_joint_literals(
    text: str,
    *,
    device_id: str,
) -> list[str]:
    """在源码/YAML 文本中找出 {device_id}_{slug}_joint_N 形式的旧写法。"""

    pattern = legacy_qualified_joint_pattern(device_id)
    return sorted(set(match.group(0) for match in pattern.finditer(text)))


def find_legacy_qualified_joint_patterns(
    text: str,
    *,
    device_id: str,
) -> list[str]:
    """找出源码里含型号 slug 的 qualified 关节模板/字面量（含 f-string）。"""

    escaped = re.escape(device_id)
    hits: set[str] = set()
    hits.update(find_legacy_qualified_joint_literals(text, device_id=device_id))
    for match in legacy_qualified_joint_pattern(device_id).finditer(text):
        hits.add(match.group(0))
    return sorted(hits)


def joint_naming_remediation(*, device_id: str, dof: int = 6) -> dict[str, object]:
    """返回领域仓应按此对齐的关节命名（只读建议，不写入磁盘）。"""

    canonical = expected_canonical_joint_names(dof=dof)
    vendor_to_canonical = {
        f"joint{index}": f"joint_{index}" for index in range(1, dof + 1)
    }
    return {
        "canonical_joint_names": list(canonical),
        "qualified_joint_names": list(
            expected_qualified_joint_names(device_id=device_id, dof=dof)
        ),
        "model_yaml": {
            "kinematic_joints": list(canonical),
            "joint_state_sources": {"canonical": list(canonical)},
        },
        "moveit_model_joint_names_mapping": vendor_to_canonical,
        "moveit_model_canonical_joint_names": list(canonical),
        "note": (
            "vendor URDF 原名（如 joint1）映射到 joint_1；"
            f"JointStateNameMap 自动生成 {device_id}_joint_N"
        ),
    }
