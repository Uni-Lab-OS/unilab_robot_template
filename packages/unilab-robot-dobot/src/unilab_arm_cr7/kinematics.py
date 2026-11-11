"""从 distribution 自带 CR7 URDF 提供确定的前向运动学。"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from unilab_robot_contracts import (
    CartesianPose,
    JointSpecification,
    JointType,
    RigidTransform,
    ToolContext,
)

_SOURCE_URDF = Path(__file__).resolve().parent / "models" / "cr7_robot.urdf"
_CANONICAL_JOINT_NAMES = tuple(f"cr7_joint_{index}" for index in range(1, 7))


@dataclass(frozen=True, slots=True)
class _KinematicJoint:
    """从 URDF 冻结的一段父坐标系、关节轴和型号限位。"""

    specification: JointSpecification
    origin: RigidTransform
    axis_xyz: tuple[float, float, float]


def load_joint_specifications() -> tuple[JointSpecification, ...]:
    """从精确 URDF 返回 CR7 六轴的类型、顺序和 SI 限位。"""

    return tuple(joint.specification for joint in _load_chain())


def forward_kinematics(
    joint_positions: Sequence[float],
    tool_context: ToolContext,
) -> CartesianPose:
    """把 CR7 六轴弧度值转换为基座坐标系中的绝对 TCP 位姿。

    参数：``joint_positions`` 按型号顺序使用弧度；``tool_context`` 提供
    末端安装面到活动 TCP 的冻结变换。
    返回：``arm_base`` 坐标系中的绝对 TCP 位姿。
    异常：关节数量或类型不符合 CR7 模型时抛出 ``ValueError``。
    """

    positions = tuple(float(value) for value in joint_positions)
    chain = _load_chain()
    if len(positions) != len(chain):
        raise ValueError(f"CR7 前向运动学必须接收 {len(chain)} 个关节值")
    transform = RigidTransform.identity()
    for position, joint in zip(positions, chain, strict=True):
        if joint.specification.joint_type not in {
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
        }:
            raise ValueError("CR7 型号运动链只允许旋转关节")
        transform = transform.compose(joint.origin).compose(
            RigidTransform.from_axis_angle(joint.axis_xyz, position)
        )
    tcp = transform.compose(tool_context.mount_to_tcp)
    return CartesianPose("arm_base", tcp.translation_m, tcp.orientation_xyzw)


@lru_cache(maxsize=1)
def _load_chain() -> tuple[_KinematicJoint, ...]:
    """解析并缓存于模块生命周期内使用的 CR7 六轴 URDF 运动链。"""

    root = ET.fromstring(_SOURCE_URDF.read_bytes())
    result: list[_KinematicJoint] = []
    for index, canonical_name in enumerate(_CANONICAL_JOINT_NAMES, start=1):
        element = root.find(f"joint[@name='joint{index}']")
        if element is None:
            raise ValueError(f"CR7 URDF 缺少 joint{index}")
        joint_type = JointType(str(element.attrib.get("type", "")))
        limit = element.find("limit")
        if limit is None:
            raise ValueError(f"CR7 URDF joint{index} 缺少限位")
        specification = JointSpecification(
            canonical_name,
            joint_type,
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
        origin = element.find("origin")
        axis = element.find("axis")
        result.append(
            _KinematicJoint(
                specification,
                RigidTransform.from_rpy(
                    _vector(origin, "xyz", (0.0, 0.0, 0.0)),
                    _vector(origin, "rpy", (0.0, 0.0, 0.0)),
                ),
                _vector(axis, "xyz", (0.0, 0.0, 1.0)),
            )
        )
    return tuple(result)


def _vector(
    element: ET.Element | None,
    attribute: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    """读取 URDF 三元属性，缺失时使用明确默认值。"""

    if element is None or attribute not in element.attrib:
        return default
    values = tuple(float(value) for value in element.attrib[attribute].split())
    if len(values) != 3:
        raise ValueError(f"CR7 URDF {attribute} 必须包含三个数值")
    return values


__all__ = ["forward_kinematics", "load_joint_specifications"]
