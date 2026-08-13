"""机械臂点位解析使用的最小刚体变换值对象。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class RigidTransform:
    """父坐标系到子坐标系的平移和四元数变换。"""

    translation_m: Vector3
    orientation_xyzw: Quaternion
    _NORMALIZATION_EPSILON: ClassVar[float] = 1e-12

    def __post_init__(self) -> None:
        """把输入冻结为有限浮点值，并归一化计算产生的四元数。

        参数：无。返回：无。
        异常：向量长度错误、包含非有限数或四元数为零时抛出 ``ValueError``。
        """

        translation = _float_tuple(self.translation_m, 3, "translation_m")
        orientation = _float_tuple(self.orientation_xyzw, 4, "orientation_xyzw")
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm <= self._NORMALIZATION_EPSILON:
            raise ValueError("orientation_xyzw 不得为零四元数")
        object.__setattr__(self, "translation_m", translation)
        object.__setattr__(
            self,
            "orientation_xyzw",
            tuple(value / norm for value in orientation),
        )

    @classmethod
    def identity(cls) -> RigidTransform:
        """返回不产生平移或旋转的单位变换。"""

        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    @classmethod
    def from_rpy(
        cls,
        translation_m: Vector3,
        rpy_rad: Vector3,
    ) -> RigidTransform:
        """从 URDF 约定的固定轴 XYZ RPY 构造刚体变换。

        参数：``translation_m`` 使用米；``rpy_rad`` 使用弧度。
        返回：等价的刚体变换。
        """

        roll, pitch, yaw = _float_tuple(rpy_rad, 3, "rpy_rad")
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return cls(
            translation_m,
            (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ),
        )

    @classmethod
    def from_axis_angle(
        cls,
        axis_xyz: Vector3,
        angle_rad: float,
    ) -> RigidTransform:
        """从单位旋转轴和弧度角构造零平移变换。

        参数：``axis_xyz`` 是关节轴；``angle_rad`` 是关节弧度值。
        返回：零平移的旋转变换。
        异常：旋转轴为零时抛出 ``ValueError``。
        """

        axis = _float_tuple(axis_xyz, 3, "axis_xyz")
        axis_norm = math.sqrt(sum(value * value for value in axis))
        if axis_norm <= cls._NORMALIZATION_EPSILON:
            raise ValueError("axis_xyz 不得为零向量")
        half = float(angle_rad) / 2.0
        scale = math.sin(half) / axis_norm
        return cls(
            (0.0, 0.0, 0.0),
            (
                axis[0] * scale,
                axis[1] * scale,
                axis[2] * scale,
                math.cos(half),
            ),
        )

    def compose(self, child: RigidTransform) -> RigidTransform:
        """把当前父变换与一个局部子变换顺序相乘。

        参数：``child`` 在当前子坐标系中表达。
        返回：与当前父坐标系同源的组合变换。
        """

        rotated = self.rotate_vector(child.translation_m)
        translated = tuple(
            self.translation_m[index] + rotated[index] for index in range(3)
        )
        return RigidTransform(
            translated,
            _quaternion_multiply(
                self.orientation_xyzw,
                child.orientation_xyzw,
            ),
        )

    def rotate_vector(self, vector: Vector3) -> Vector3:
        """使用当前四元数旋转一个三维向量。

        参数：``vector`` 在当前子坐标系中表达。
        返回：在当前父坐标系中表达的向量。
        """

        candidate = _float_tuple(vector, 3, "vector")
        quaternion = self.orientation_xyzw
        pure = (candidate[0], candidate[1], candidate[2], 0.0)
        rotated = _quaternion_multiply_raw(
            _quaternion_multiply_raw(quaternion, pure),
            _quaternion_conjugate(quaternion),
        )
        return (rotated[0], rotated[1], rotated[2])


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """组合两个旋转并返回归一化 XYZW 四元数。

    参数：``left`` 先作用于父坐标系；``right`` 是后续局部旋转。
    返回：归一化后的组合旋转。
    """

    return RigidTransform(
        (0.0, 0.0, 0.0),
        _quaternion_multiply(left, right),
    ).orientation_xyzw


def _quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """组合两个旋转四元数，并通过值对象完成归一化。"""

    return _quaternion_multiply_raw(
        _float_tuple(left, 4, "left quaternion"),
        _float_tuple(right, 4, "right quaternion"),
    )


def _quaternion_multiply_raw(left: Quaternion, right: Quaternion) -> Quaternion:
    """执行不归一化的 XYZW Hamilton 乘法。"""

    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quaternion_conjugate(value: Quaternion) -> Quaternion:
    """返回单位 XYZW 四元数的共轭。"""

    x, y, z, w = value
    return (-x, -y, -z, w)


def _float_tuple(value: object, length: int, field: str) -> tuple[float, ...]:
    """把序列验证并转换为固定长度有限浮点元组。"""

    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field} 必须是长度 {length} 的数值序列")
    try:
        normalized = tuple(float(item) for item in value)  # type: ignore[union-attr]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是长度 {length} 的数值序列") from exc
    if len(normalized) != length or not all(math.isfinite(item) for item in normalized):
        raise ValueError(f"{field} 必须包含 {length} 个有限数值")
    return normalized


__all__ = ["Quaternion", "RigidTransform", "Vector3", "quaternion_multiply"]
