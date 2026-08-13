"""机械臂维护界面输入的笛卡尔位姿与欧拉角规范化合同。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .geometry import RigidTransform, quaternion_multiply
from .targets import CartesianPose

EULER_ROTATION_CONVENTION = "extrinsic_fixed_axes"


class AngleUnit(str, Enum):
    """维护界面允许选择的角度显示与输入单位。"""

    DEG = "deg"
    RAD = "rad"

    def to_radians(self, value: float) -> float:
        """把一个有限角度值转换为弧度。

        参数：``value`` 是按当前枚举单位表达的角度。
        返回：供统一协议和运动 Adapter 使用的弧度值。
        异常：输入不是有限数时抛出 ``ValueError``。
        """

        try:
            normalized = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("角度值必须是有限数") from exc
        if not math.isfinite(normalized):
            raise ValueError("角度值必须是有限数")
        return math.radians(normalized) if self is AngleUnit.DEG else normalized


class EulerRotationOrder(str, Enum):
    """维护界面支持的六种固定轴欧拉旋转应用顺序。"""

    XYZ = "xyz"
    XZY = "xzy"
    YZX = "yzx"
    YXZ = "yxz"
    ZXY = "zxy"
    ZYX = "zyx"


@dataclass(frozen=True, slots=True)
class CommissioningPoseInput:
    """操作员输入的一次临时绝对 TCP 目标位姿。

    ``xyz_mm`` 保留维护界面常用的毫米输入；``rotation_xyz`` 始终按
    ``(rx, ry, rz)`` 命名三个固定坐标轴的角度，``rotation_order`` 只决定
    这三个旋转的应用先后。转换后的目标使用 m 与 XYZW 四元数，避免各厂商
    Adapter 重复解释欧拉角。
    """

    frame_ref: str
    xyz_mm: tuple[float, float, float]
    rotation_xyz: tuple[float, float, float]
    angle_unit: AngleUnit
    rotation_order: EulerRotationOrder

    def __post_init__(self) -> None:
        """校验坐标系、三维位置、三轴角度、角度单位和旋转顺序。"""

        if not self.frame_ref.strip():
            raise ValueError("调试目标位姿必须包含 frame_ref")
        object.__setattr__(self, "xyz_mm", _finite_tuple(self.xyz_mm, "xyz_mm"))
        object.__setattr__(
            self,
            "rotation_xyz",
            _finite_tuple(self.rotation_xyz, "rotation_xyz"),
        )
        try:
            object.__setattr__(self, "angle_unit", AngleUnit(self.angle_unit))
            object.__setattr__(
                self,
                "rotation_order",
                EulerRotationOrder(self.rotation_order),
            )
        except ValueError as exc:
            raise ValueError("调试目标位姿的角度单位或旋转顺序无效") from exc

    @property
    def xyz_m(self) -> tuple[float, float, float]:
        """把界面毫米位置转换为协议统一使用的米。

        返回：按 ``frame_ref`` 表达的三维米制绝对位置。
        """

        return tuple(value / 1000.0 for value in self.xyz_mm)

    @property
    def rotation_rad_xyz(self) -> tuple[float, float, float]:
        """把 ``rx/ry/rz`` 转换为按轴命名的弧度值。

        返回：顺序固定为 X、Y、Z 的三轴弧度值；应用顺序仍由
        ``rotation_order`` 单独决定。
        """

        return tuple(self.angle_unit.to_radians(value) for value in self.rotation_xyz)

    @property
    def orientation_xyzw(self) -> tuple[float, float, float, float]:
        """按固定父坐标轴顺序生成归一化 XYZW 四元数。

        返回：与界面欧拉角输入等价、可交给后续 Adapter 的单位四元数。
        """

        angle_by_axis = dict(zip("xyz", self.rotation_rad_xyz, strict=True))
        axis_vector = {
            "x": (1.0, 0.0, 0.0),
            "y": (0.0, 1.0, 0.0),
            "z": (0.0, 0.0, 1.0),
        }
        orientation = (0.0, 0.0, 0.0, 1.0)
        for axis in self.rotation_order.value:
            axis_rotation = RigidTransform.from_axis_angle(
                axis_vector[axis],
                angle_by_axis[axis],
            ).orientation_xyzw
            orientation = quaternion_multiply(axis_rotation, orientation)
        return orientation

    @property
    def resolved_pose(self) -> CartesianPose:
        """生成已经统一单位和旋转表达的绝对 TCP 位姿。

        返回：使用 m 和 XYZW 四元数的 ``CartesianPose``。
        """

        return CartesianPose(self.frame_ref, self.xyz_m, self.orientation_xyzw)


def _finite_tuple(
    value: object,
    field_name: str,
) -> tuple[float, float, float]:
    """把维护输入校验为包含三个有限浮点数的元组。

    参数：``value`` 是待校验序列；``field_name`` 是错误信息中的字段名。
    返回：长度恰好为三的浮点元组。
    异常：输入不可迭代、长度错误或包含非有限数时抛出 ``ValueError``。
    """

    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field_name} 必须包含三个有限数")
    try:
        normalized = tuple(float(item) for item in value)  # type: ignore[union-attr]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 必须包含三个有限数") from exc
    if len(normalized) != 3 or not all(math.isfinite(item) for item in normalized):
        raise ValueError(f"{field_name} 必须包含三个有限数")
    return normalized  # type: ignore[return-value]


__all__ = [
    "AngleUnit",
    "CommissioningPoseInput",
    "EULER_ROTATION_CONVENTION",
    "EulerRotationOrder",
]
