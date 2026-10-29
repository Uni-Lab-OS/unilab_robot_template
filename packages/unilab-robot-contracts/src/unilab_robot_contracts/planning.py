"""厂商无关的路径规划请求、结果与规划器端口。

该模块只表达“把什么场景、从哪个状态、按什么约束规划到哪里”以及
“规划器返回了什么”。它不导入 MoveIt、ROS、FCL、OMPL 或 NumPy，也不
包含设备包的物料、工站或接触语义。具体后端（``MoveItPlanner`` 等）负责
把这里的值对象翻译成自己的原生类型。

设计约束：

* 关节值使用弧度，导轨使用米，时间使用秒，位姿使用父坐标系中的米和
  XYZW 四元数；
* 所有数值必须是有限数，所有矩阵必须是刚体变换；
* 成功结果必须包含非空、有序且带时间的路点；
* 规划器身份和结果摘要进入结果，便于跨进程、跨机器核对同一输入；
* 空碰撞世界是合法的（自由空间规划），但空成功轨迹永远不合法。

本模块没有运行时第三方依赖，可在没有 ROS/MoveIt 的环境中导入和测试。
"""

from __future__ import annotations
from .mesh import PackedMeshRows


import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal, Protocol, TypeAlias, runtime_checkable

from .attachments import PayloadCollisionProfile

PlanStatus: TypeAlias = Literal[
    "success", "invalid", "collision", "unreachable", "timeout"
]
PLAN_STATUSES: frozenset[str] = frozenset(
    {"success", "invalid", "collision", "unreachable", "timeout"}
)
WaypointSource: TypeAlias = Literal[
    "planner", "rail", "safe_pose", "seed", "recovery"
]

_SCHEMA = "unilab.planning/v1"
_MATRIX_ATOL = 1e-9
_MAX_TIMEOUT_S = 3600.0
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_TWO_PI = 2.0 * math.pi
#: 载荷基元必须在载荷体坐标系对齐：``CollisionGeometry`` 没有局部位姿。
_PRIMITIVE_POSE_ATOL = 1e-9
#: Profile 基元与碰撞世界几何必须是同一组保守形状。
_GEOMETRY_SIZE_ATOL = 1e-9


def _text(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} 必须是字符串")
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{name} 不能为空")
    return normalized


def _finite(value: object, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, (str, bytes)) or isinstance(value, bool):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        normalized = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须是有限数值") from error
    if not math.isfinite(normalized):
        raise ValueError(f"{name} 必须是有限数值")
    if minimum is not None and normalized < minimum:
        raise ValueError(f"{name} 不得小于 {minimum}")
    return normalized


def _float_tuple(value: object, length: int, name: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)):
        raise TypeError(f"{name} 必须是长度 {length} 的数值序列")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise TypeError(f"{name} 必须是长度 {length} 的数值序列") from error
    if any(isinstance(item, bool) for item in items):
        raise ValueError(f"{name} 不得包含布尔值")
    try:
        normalized = tuple(float(item) for item in items)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须是长度 {length} 的有限数值序列") from error
    if len(normalized) != length or not all(
        math.isfinite(item) for item in normalized
    ):
        raise ValueError(f"{name} 必须是长度 {length} 的有限数值序列")
    return normalized


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} 必须是映射")
    normalized: dict[str, Any] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise TypeError(f"{name} 的键必须是字符串")
        normalized[key] = item
    return normalized


def _canonical_json(value: object) -> str:
    try:
        return json.dumps(
            value,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError) as error:
        raise ValueError("规划值对象必须可规范化为 JSON") from error


def _digest(value: object) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _matrix4(value: object, name: str) -> tuple[tuple[float, float, float, float], ...]:
    if isinstance(value, (str, bytes)):
        raise TypeError(f"{name} 必须是 4x4 数值矩阵")
    try:
        raw_rows = tuple(tuple(row) for row in value)  # type: ignore[union-attr]
    except TypeError as error:
        raise TypeError(f"{name} 必须是 4x4 数值矩阵") from error
    if any(isinstance(item, bool) for row in raw_rows for item in row):
        raise ValueError(f"{name} 不得包含布尔值")
    try:
        rows = tuple(tuple(float(item) for item in row) for row in raw_rows)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须是 4x4 数值矩阵") from error
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError(f"{name} 必须是 4x4 数值矩阵")
    if not all(math.isfinite(item) for row in rows for item in row):
        raise ValueError(f"{name} 必须只包含有限数值")
    if max(abs(rows[3][index] - float(index == 3)) for index in range(4)) > _MATRIX_ATOL:
        raise ValueError(f"{name} 的末行必须是 [0, 0, 0, 1]")
    for left in range(3):
        for right in range(3):
            dot = sum(rows[index][left] * rows[index][right] for index in range(3))
            if abs(dot - float(left == right)) > 1e-7:
                raise ValueError(f"{name} 的旋转部分必须是正交矩阵")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if determinant < 0.0:
        raise ValueError(f"{name} 不得包含反射变换")
    return rows


def _optional_mapping(value: object, name: str) -> dict[str, Any]:
    if value is None:
        return {}
    return _mapping(value, name)


def _is_identity_transform(
    translation_m: Sequence[float], orientation_xyzw: Sequence[float]
) -> bool:
    """返回刚体变换是否为单位变换（四元数允许 +/- 两种符号）。"""

    if any(abs(float(value)) > _PRIMITIVE_POSE_ATOL for value in translation_m):
        return False
    quaternion = tuple(float(value) for value in orientation_xyzw)
    identity = (0.0, 0.0, 0.0, 1.0)
    delta = max(
        abs(quaternion[index] - identity[index]) for index in range(4)
    )
    flipped = max(
        abs(quaternion[index] + identity[index]) for index in range(4)
    )
    return min(delta, flipped) <= _PRIMITIVE_POSE_ATOL


def _geometry_present(
    geometries: Sequence[CollisionGeometry], wanted: CollisionGeometry
) -> bool:
    """返回碰撞对象几何里是否存在与 ``wanted`` 等价的保守形状。"""

    for geometry in geometries:
        if geometry.kind != wanted.kind:
            continue
        if wanted.kind == "box":
            size = wanted.size_m or ()
            candidate = geometry.size_m or ()
            if len(candidate) == len(size) and all(
                abs(left - right) <= _GEOMETRY_SIZE_ATOL
                for left, right in zip(candidate, size)
            ):
                return True
        elif wanted.kind == "cylinder":
            if (
                geometry.axis == wanted.axis
                and abs(float(geometry.radius_m or 0.0) - float(wanted.radius_m or 0.0))
                <= _GEOMETRY_SIZE_ATOL
                and abs(float(geometry.length_m or 0.0) - float(wanted.length_m or 0.0))
                <= _GEOMETRY_SIZE_ATOL
            ):
                return True
    return False


@dataclass(frozen=True, slots=True)
class RobotModel:
    """一次规划冻结的机器人运动学模型。

    ``urdf_xml``/``srdf_xml`` 是可移植的 XML 文本，不依赖本机路径。
    ``kinematics`` 是 MoveIt 风格的求解器参数；没有该字段时后端可使用
    自己的默认值，但不得在规划成功后声称使用了未记录的求解器。
    """

    model_id: str
    urdf_xml: str
    srdf_xml: str
    base_frame: str
    flange_link: str
    planning_group: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...] | None = None
    kinematics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "model_id", _text(self.model_id, "model_id"))
        object.__setattr__(self, "base_frame", _text(self.base_frame, "base_frame"))
        object.__setattr__(self, "flange_link", _text(self.flange_link, "flange_link"))
        object.__setattr__(
            self, "planning_group", _text(self.planning_group, "planning_group")
        )
        for attr, label in (("urdf_xml", "URDF"), ("srdf_xml", "SRDF")):
            raw = getattr(self, attr)
            if not isinstance(raw, str) or not raw.strip():
                raise ValueError(f"RobotModel.{attr} 不能为空")
            try:
                root = ET.fromstring(raw)
            except ET.ParseError as error:
                raise ValueError(f"RobotModel.{attr} 不是合法 {label} XML") from error
            if root.tag.lower() not in {"robot", "robot_description"}:
                raise ValueError(f"RobotModel.{attr} 根元素不是 robot")
        if isinstance(self.joint_names, (str, bytes)):
            raise TypeError("joint_names 必须是关节名序列")
        names = tuple(str(name).strip() for name in self.joint_names)
        if not names or any(not name for name in names):
            raise ValueError("joint_names 不得为空或包含空名称")
        if len(set(names)) != len(names):
            raise ValueError("joint_names 不得重复")
        object.__setattr__(self, "joint_names", names)
        if self.joint_limits is None:
            object.__setattr__(self, "joint_limits", None)
        else:
            if len(self.joint_limits) != len(names):
                raise ValueError("joint_limits 必须与 joint_names 一一对应")
            limits: list[tuple[float, float]] = []
            for index, (lower, upper) in enumerate(self.joint_limits):
                low = _finite(lower, f"joint_limits[{index}].lower")
                high = _finite(upper, f"joint_limits[{index}].upper")
                if low > high:
                    raise ValueError(f"关节 {names[index]} 下限不得大于上限")
                limits.append((low, high))
            object.__setattr__(self, "joint_limits", tuple(limits))
        object.__setattr__(self, "kinematics", _mapping(self.kinematics, "kinematics"))

    @property
    def dof(self) -> int:
        """返回规划组关节数。"""

        return len(self.joint_names)

    def validate_joint_positions(
        self, positions: object, *, name: str = "joint_positions"
    ) -> tuple[float, ...]:
        """校验一组关节值，并在声明了限位时逐轴检查。"""

        values = _float_tuple(positions, self.dof, name)
        for index, (value, limit) in enumerate(zip(values, self.joint_limits or ())):
            if not limit[0] <= value <= limit[1]:
                raise ValueError(
                    f"{name}[{index}]={value} 超出 [{limit[0]}, {limit[1]}]"
                )
        return values

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的显式模型字段。"""

        return {
            "model_id": self.model_id,
            "base_frame": self.base_frame,
            "flange_link": self.flange_link,
            "planning_group": self.planning_group,
            "joint_names": list(self.joint_names),
            "joint_limits": [list(limit) for limit in self.joint_limits]
            if self.joint_limits is not None
            else None,
            "urdf_sha256": hashlib.sha256(self.urdf_xml.encode("utf-8")).hexdigest(),
            "srdf_sha256": hashlib.sha256(self.srdf_xml.encode("utf-8")).hexdigest(),
        }

    @property
    def digest(self) -> str:
        """返回模型 XML、关节顺序和规划组的稳定摘要。"""

        return _digest(self.as_mapping())


@dataclass(frozen=True, slots=True)
class RailAxis:
    """独立于六轴规划组的线性导轨轴。"""

    axis_id: str
    frame: str
    limits_m: tuple[float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "axis_id", _text(self.axis_id, "axis_id"))
        object.__setattr__(self, "frame", _text(self.frame, "frame"))
        if isinstance(self.limits_m, (str, bytes)) or len(self.limits_m) != 2:
            raise ValueError("RailAxis.limits_m 必须是 (lower, upper)")
        lower = _finite(self.limits_m[0], "limits_m.lower")
        upper = _finite(self.limits_m[1], "limits_m.upper")
        if lower > upper:
            raise ValueError("RailAxis 下限不得大于上限")
        object.__setattr__(self, "limits_m", (lower, upper))

    def contains(self, value: object) -> bool:
        """返回导轨位置是否位于闭区间内。"""

        try:
            position = _finite(value, "rail_m")
        except (TypeError, ValueError):
            return False
        return self.limits_m[0] <= position <= self.limits_m[1]

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的导轨字段。"""

        return {
            "axis_id": self.axis_id,
            "frame": self.frame,
            "unit": "m",
            "limits_m": list(self.limits_m),
        }


@dataclass(frozen=True, slots=True)
class CartesianGoal:
    """法兰（或指定 link）的绝对笛卡尔目标位姿。"""

    frame_ref: str
    xyz_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    link_name: str | None = None
    position_tolerance_m: float = 1e-3
    orientation_tolerance_rad: float = 1e-3

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_ref", _text(self.frame_ref, "frame_ref"))
        if self.link_name is not None:
            object.__setattr__(self, "link_name", _text(self.link_name, "link_name"))
        xyz = _float_tuple(self.xyz_m, 3, "xyz_m")
        quaternion = _float_tuple(self.orientation_xyzw, 4, "orientation_xyzw")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation_xyzw 不得为零四元数")
        object.__setattr__(self, "xyz_m", xyz)  # type: ignore[arg-type]
        object.__setattr__(
            self,
            "orientation_xyzw",
            tuple(value / norm for value in quaternion),  # type: ignore[arg-type]
        )
        object.__setattr__(
            self,
            "position_tolerance_m",
            _finite(
                self.position_tolerance_m, "position_tolerance_m", minimum=0.0
            ),
        )
        object.__setattr__(
            self,
            "orientation_tolerance_rad",
            _finite(
                self.orientation_tolerance_rad,
                "orientation_tolerance_rad",
                minimum=0.0,
            ),
        )

    @property
    def transform(self) -> tuple[tuple[float, float, float, float], ...]:
        """返回与该位姿等价的 4x4 刚体矩阵（不使用 scipy）。"""

        x, y, z, w = self.orientation_xyzw
        return (
            (
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
                self.xyz_m[0],
            ),
            (
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
                self.xyz_m[1],
            ),
            (
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
                self.xyz_m[2],
            ),
            (0.0, 0.0, 0.0, 1.0),
        )

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的位姿字段。"""

        return {
            "frame_ref": self.frame_ref,
            "link_name": self.link_name,
            "xyz_m": list(self.xyz_m),
            "orientation_xyzw": list(self.orientation_xyzw),
            "position_tolerance_m": self.position_tolerance_m,
            "orientation_tolerance_rad": self.orientation_tolerance_rad,
        }


@dataclass(frozen=True, slots=True)
class PlanGoal:
    """联合变量目标或绝对位姿目标；两者只能出现一个。"""

    kind: Literal["joint", "pose"]
    joint_positions: tuple[float, ...] | None = None
    pose: CartesianGoal | None = None

    def __post_init__(self) -> None:
        if self.kind not in ("joint", "pose"):
            raise ValueError("PlanGoal.kind 只允许 joint/pose")
        if self.kind == "joint":
            if self.joint_positions is None or self.pose is not None:
                raise ValueError("关节目标必须且只能提供 joint_positions")
            if isinstance(self.joint_positions, (str, bytes)):
                raise TypeError("joint_positions 必须是数值序列")
            values = tuple(
                _finite(item, f"joint_positions[{index}]")
                for index, item in enumerate(self.joint_positions)
            )
            if not values:
                raise ValueError("joint_positions 必须是非空有限数值序列")
            object.__setattr__(self, "joint_positions", values)
        else:
            if self.pose is None or self.joint_positions is not None:
                raise ValueError("位姿目标必须且只能提供 pose")
            if not isinstance(self.pose, CartesianGoal):
                raise TypeError("PlanGoal.pose 必须是 CartesianGoal")

    @classmethod
    def from_joint(cls, positions: Sequence[float]) -> PlanGoal:
        """构造关节目标。"""

        return cls("joint", joint_positions=tuple(positions))

    @classmethod
    def from_pose(cls, pose: CartesianGoal) -> PlanGoal:
        """构造绝对位姿目标。"""

        return cls("pose", pose=pose)

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的目标字段。"""

        if self.kind == "joint":
            return {"kind": "joint", "joint_positions": list(self.joint_positions or ())}
        assert self.pose is not None
        return {"kind": "pose", "pose": self.pose.as_mapping()}


@dataclass(frozen=True, slots=True)
class OrientationConstraintSpec:
    """路径上的绝对姿态约束；参考坐标系和 link 必须显式给出。"""

    frame_ref: str
    link_name: str
    orientation_xyzw: tuple[float, float, float, float]
    absolute_x_axis_tolerance_rad: float = 0.05
    absolute_y_axis_tolerance_rad: float = 0.05
    absolute_z_axis_tolerance_rad: float = math.pi
    weight: float = 1.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_ref", _text(self.frame_ref, "frame_ref"))
        object.__setattr__(self, "link_name", _text(self.link_name, "link_name"))
        quaternion = _float_tuple(self.orientation_xyzw, 4, "orientation_xyzw")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-12:
            raise ValueError("orientation_xyzw 不得为零四元数")
        object.__setattr__(
            self,
            "orientation_xyzw",
            tuple(value / norm for value in quaternion),  # type: ignore[arg-type]
        )
        for attr in (
            "absolute_x_axis_tolerance_rad",
            "absolute_y_axis_tolerance_rad",
            "absolute_z_axis_tolerance_rad",
        ):
            object.__setattr__(
                self, attr, _finite(getattr(self, attr), attr, minimum=0.0)
            )
        object.__setattr__(self, "weight", _finite(self.weight, "weight", minimum=0.0))

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的约束字段。"""

        return {
            "frame_ref": self.frame_ref,
            "link_name": self.link_name,
            "orientation_xyzw": list(self.orientation_xyzw),
            "absolute_x_axis_tolerance_rad": self.absolute_x_axis_tolerance_rad,
            "absolute_y_axis_tolerance_rad": self.absolute_y_axis_tolerance_rad,
            "absolute_z_axis_tolerance_rad": self.absolute_z_axis_tolerance_rad,
            "weight": self.weight,
        }


@dataclass(frozen=True, slots=True)
class SafePoseConstraint:
    """规划必须经过的安全关节姿态；后端可拆成两段规划。"""

    joint_positions: tuple[float, ...]
    tolerance_rad: float = 1e-3

    def __post_init__(self) -> None:
        if isinstance(self.joint_positions, (str, bytes)):
            raise TypeError("safe_pose.joint_positions 必须是数值序列")
        values = tuple(
            _finite(item, f"safe_pose.joint_positions[{index}]")
            for index, item in enumerate(self.joint_positions)
        )
        if not values:
            raise ValueError("safe_pose 必须是非空有限关节序列")
        object.__setattr__(self, "joint_positions", values)
        object.__setattr__(
            self, "tolerance_rad", _finite(self.tolerance_rad, "tolerance_rad", minimum=0.0)
        )


@dataclass(frozen=True, slots=True)
class ClearanceConstraint:
    """离最近碰撞体的最小间隙；后端必须显式声明是否原生支持。"""

    min_clearance_m: float
    object_ids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "min_clearance_m",
            _finite(self.min_clearance_m, "min_clearance_m", minimum=0.0),
        )
        if isinstance(self.object_ids, (str, bytes)):
            raise TypeError("clearance.object_ids 必须是对象 id 序列")
        ids = tuple(str(item).strip() for item in self.object_ids)
        if any(not item for item in ids) or len(set(ids)) != len(ids):
            raise ValueError("clearance.object_ids 不得为空、重复或包含空字符串")
        object.__setattr__(self, "object_ids", ids)


@dataclass(frozen=True, slots=True)
class PlanConstraints:
    """一次规划的路径约束集合。"""

    orientation: tuple[OrientationConstraintSpec, ...] = ()
    safe_pose: SafePoseConstraint | None = None
    clearance: ClearanceConstraint | None = None
    allowed_collision_pairs: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.orientation, (str, bytes)):
            raise TypeError("orientation 必须是约束序列")
        orientation = tuple(self.orientation)
        if any(not isinstance(item, OrientationConstraintSpec) for item in orientation):
            raise TypeError("orientation 只能包含 OrientationConstraintSpec")
        object.__setattr__(self, "orientation", orientation)
        if self.safe_pose is not None and not isinstance(
            self.safe_pose, SafePoseConstraint
        ):
            raise TypeError("safe_pose 必须是 SafePoseConstraint")
        if self.clearance is not None and not isinstance(
            self.clearance, ClearanceConstraint
        ):
            raise TypeError("clearance 必须是 ClearanceConstraint")
        pairs: list[tuple[str, str]] = []
        for index, pair in enumerate(self.allowed_collision_pairs):
            if isinstance(pair, (str, bytes)) or len(pair) != 2:
                raise ValueError(f"allowed_collision_pairs[{index}] 必须是二元组")
            left = _text(pair[0], f"allowed_collision_pairs[{index}].a")
            right = _text(pair[1], f"allowed_collision_pairs[{index}].b")
            if left == right:
                raise ValueError("allowed_collision_pairs 不得引用同一对象")
            normalized = tuple(sorted((left, right)))
            if normalized in pairs:
                raise ValueError("allowed_collision_pairs 不得重复")
            pairs.append(normalized)  # type: ignore[arg-type]
        object.__setattr__(self, "allowed_collision_pairs", tuple(pairs))

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的约束字段。"""

        return {
            "orientation": [item.as_mapping() for item in self.orientation],
            "safe_pose": {
                "joint_positions": list(self.safe_pose.joint_positions),
                "tolerance_rad": self.safe_pose.tolerance_rad,
            }
            if self.safe_pose is not None
            else None,
            "clearance": {
                "min_clearance_m": self.clearance.min_clearance_m,
                "object_ids": list(self.clearance.object_ids),
            }
            if self.clearance is not None
            else None,
            "allowed_collision_pairs": [list(pair) for pair in self.allowed_collision_pairs],
        }


@dataclass(frozen=True, slots=True)
class CollisionGeometry:
    """一个碰撞几何图元。

    支持三种精确表示：文件网格、内存三角网格、盒体和圆柱。内存网格的
    顶点使用米，索引是三角面顶点下标；该形式使 CAD 导出无需经过私有
    PhysX 烹饪结果。具体后端若不支持某种图元，必须在 ``validate`` 或
    ``plan`` 中显式拒绝，不能静默忽略。
    """

    kind: Literal["mesh", "box", "cylinder"]
    mesh_path: str | None = None
    vertices: tuple[tuple[float, float, float], ...] | PackedMeshRows | None = None
    triangles: tuple[tuple[int, int, int], ...] | PackedMeshRows | None = None
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    size_m: tuple[float, float, float] | None = None
    radius_m: float | None = None
    length_m: float | None = None
    axis: Literal["x", "y", "z"] = "z"
    _mesh_digest: str | None = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.kind not in ("mesh", "box", "cylinder"):
            raise ValueError("CollisionGeometry.kind 只允许 mesh/box/cylinder")
        object.__setattr__(
            self, "mesh_scale", _float_tuple(self.mesh_scale, 3, "mesh_scale")
        )
        if any(value <= 0.0 for value in self.mesh_scale):
            raise ValueError("mesh_scale 必须为正有限数")
        if self.kind == "mesh":
            if self.size_m is not None or self.radius_m is not None or self.length_m is not None:
                raise ValueError("mesh 几何不得声明盒体或圆柱参数")
            if self.mesh_path is not None:
                object.__setattr__(self, "mesh_path", _text(self.mesh_path, "mesh_path"))
            if (self.vertices is None) != (self.triangles is None):
                raise ValueError("内存 mesh 必须同时提供 vertices 和 triangles")
            if self.vertices is not None:
                if not self.vertices or not self.triangles:
                    raise ValueError("内存 mesh 必须包含顶点和三角面")
                packed=isinstance(self.vertices,PackedMeshRows) and isinstance(self.triangles,PackedMeshRows)
                if packed:
                    if self.vertices.code!="d" or self.triangles.code!="q":
                        raise ValueError("Packed mesh coordinate/index types differ")
                    if any(not math.isfinite(v) for v in memoryview(self.vertices.data).cast("d")):
                        raise ValueError("Nonfinite packed mesh vertex")
                    vertices=self.vertices
                else:
                    vertices = tuple(
                        _float_tuple(vertex, 3, f"vertices[{index}]")
                        for index, vertex in enumerate(self.vertices)
                    )
                if packed:
                    indices=memoryview(self.triangles.data).cast("q")
                    if min(indices)<0 or max(indices)>=len(vertices):
                        raise ValueError("triangles 索引越界")
                    if any(a==b or a==c or b==c for a,b,c in self.triangles):
                        raise ValueError("triangles 不得退化")
                    triangles=[]
                else:
                    triangles: list[tuple[int, int, int]] = []
                    for index, triangle in enumerate(self.triangles):
                        if (
                            isinstance(triangle, (str, bytes))
                            or len(triangle) != 3
                            or any(
                                isinstance(item, bool) or not isinstance(item, int)
                                for item in triangle
                            )
                        ):
                            raise ValueError(f"triangles[{index}] 必须是三个整数索引")
                        a, b, c = triangle
                        if not (0 <= a < len(vertices) and 0 <= b < len(vertices) and 0 <= c < len(vertices)):
                            raise ValueError(f"triangles[{index}] 索引越界")
                        if len({a, b, c}) != 3:
                            raise ValueError(f"triangles[{index}] 不得退化")
                        if not packed:triangles.append((a, b, c))
                object.__setattr__(self, "vertices", vertices)
                object.__setattr__(self, "triangles", self.triangles if packed else tuple(triangles))
            elif self.mesh_path is None:
                raise ValueError("mesh 几何必须提供 mesh_path 或内存顶点/三角面")
        elif self.kind == "box":
            if self.mesh_path or self.vertices is not None or self.triangles is not None:
                raise ValueError("box 几何不得声明 mesh 字段")
            size = _float_tuple(self.size_m, 3, "size_m")
            if any(value <= 0.0 for value in size):
                raise ValueError("size_m 必须为正有限数")
            object.__setattr__(self, "size_m", size)
        else:
            if self.mesh_path or self.vertices is not None or self.triangles is not None:
                raise ValueError("cylinder 几何不得声明 mesh 字段")
            if self.axis not in ("x", "y", "z"):
                raise ValueError("cylinder.axis 只允许 x/y/z")
            object.__setattr__(
                self, "radius_m", _finite(self.radius_m, "radius_m", minimum=1e-9)
            )
            object.__setattr__(
                self, "length_m", _finite(self.length_m, "length_m", minimum=1e-9)
            )

    def as_mapping(self) -> dict[str, Any]:
        """返回不带网格顶点明细的摘要字段。"""

        result: dict[str, Any] = {"kind": self.kind}
        if self.kind == "mesh":
            result["mesh_path"] = self.mesh_path
            result["mesh_scale"] = list(self.mesh_scale)
            if self.vertices is not None:
                result["vertex_count"] = len(self.vertices)
                result["triangle_count"] = len(self.triangles or ())
                if self._mesh_digest is None:
                    digest = hashlib.sha256()
                    from itertools import islice
                    for rows in (self.vertices,self.triangles or ()):
                        iterator=iter(rows)
                        while batch:=list(islice(iterator,8192)):
                            # 与逐行 JSON 串联完全相同，仅批量调用编码器。
                            text=_canonical_json(batch)[1:-1].replace("],[","][")
                            digest.update(text.encode("ascii"))
                    object.__setattr__(self, "_mesh_digest", digest.hexdigest())
                result["mesh_sha256"] = self._mesh_digest
        elif self.kind == "box":
            result["size_m"] = list(self.size_m or ())
        else:
            result.update(
                radius_m=self.radius_m, length_m=self.length_m, axis=self.axis
            )
        return result


@dataclass(frozen=True, slots=True)
class CollisionObject:
    """碰撞世界中的一个刚体对象。"""

    object_id: str
    world_pose: tuple[tuple[float, float, float, float], ...]
    geometry: tuple[CollisionGeometry, ...]
    role: str = "world"
    attached_to_link: str | None = None
    touch_links: tuple[str, ...] = ()
    geometry_poses: tuple[tuple[tuple[float, ...], ...], ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "object_id", _text(self.object_id, "object_id"))
        object.__setattr__(self, "world_pose", _matrix4(self.world_pose, "world_pose"))
        object.__setattr__(self, "role", _text(self.role, "role"))
        if isinstance(self.geometry, (str, bytes)):
            raise TypeError("geometry 必须是 CollisionGeometry 序列")
        geometry = tuple(self.geometry)
        if not geometry:
            raise ValueError(f"对象 {self.object_id} 至少需要一个碰撞几何")
        if any(not isinstance(item, CollisionGeometry) for item in geometry):
            raise TypeError("geometry 只能包含 CollisionGeometry")
        object.__setattr__(self, "geometry", geometry)
        poses = tuple(_matrix4(pose, "geometry_pose") for pose in self.geometry_poses)
        if poses and len(poses) != len(geometry):
            raise ValueError("geometry_poses must match geometry length")
        object.__setattr__(self, "geometry_poses", poses)
        if self.attached_to_link is not None:
            object.__setattr__(
                self, "attached_to_link", _text(self.attached_to_link, "attached_to_link")
            )
        links = tuple(str(item).strip() for item in self.touch_links)
        if any(not item for item in links) or len(set(links)) != len(links):
            raise ValueError("touch_links 不得重复或包含空字符串")
        object.__setattr__(self, "touch_links", links)

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的对象字段。"""

        return {
            "object_id": self.object_id,
            "role": self.role,
            "world_pose": [list(row) for row in self.world_pose],
            "attached_to_link": self.attached_to_link,
            "touch_links": list(self.touch_links),
            "geometry": [item.as_mapping() for item in self.geometry],
            **({"geometry_poses": self.geometry_poses} if self.geometry_poses else {}),
        }


@dataclass(frozen=True, slots=True)
class CollisionWorld:
    """规划时刻的完整碰撞世界；空对象集合表示自由空间。"""

    frame_id: str = "world"
    objects: tuple[CollisionObject, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame_id", _text(self.frame_id, "frame_id"))
        if isinstance(self.objects, (str, bytes)):
            raise TypeError("objects 必须是 CollisionObject 序列")
        objects = tuple(self.objects)
        if any(not isinstance(item, CollisionObject) for item in objects):
            raise TypeError("objects 只能包含 CollisionObject")
        ids = [item.object_id for item in objects]
        if len(set(ids)) != len(ids):
            raise ValueError("碰撞世界对象 id 不得重复")
        object.__setattr__(self, "objects", objects)

    @property
    def object_ids(self) -> tuple[str, ...]:
        """按声明顺序返回对象 id。"""

        return tuple(item.object_id for item in self.objects)

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的世界字段。"""

        return {
            "frame_id": self.frame_id,
            "objects": [item.as_mapping() for item in self.objects],
        }

    @property
    def digest(self) -> str:
        """返回世界摘要。"""

        return _digest(self.as_mapping())


@dataclass(frozen=True, slots=True)
class AttachmentSpec:
    """工具/载荷相对法兰的固定安装关系。"""

    body_id: str
    link_name: str
    flange_from_body: tuple[tuple[float, float, float, float], ...]
    touch_links: tuple[str, ...] = ()
    purpose: Literal["tool_or_payload", "robot_fixture"] = "tool_or_payload"

    def __post_init__(self) -> None:
        if self.purpose not in ("tool_or_payload","robot_fixture"):
            raise ValueError("Unknown attachment purpose")
        object.__setattr__(self, "body_id", _text(self.body_id, "body_id"))
        object.__setattr__(self, "link_name", _text(self.link_name, "link_name"))
        object.__setattr__(
            self,
            "flange_from_body",
            _matrix4(self.flange_from_body, "flange_from_body"),
        )
        if isinstance(self.touch_links, (str, bytes)):
            raise TypeError("touch_links 必须是 link 序列")
        links = tuple(str(item).strip() for item in self.touch_links)
        if any(not item for item in links) or len(set(links)) != len(links):
            raise ValueError("touch_links 不得重复或包含空字符串")
        object.__setattr__(self, "touch_links", links)


@dataclass(frozen=True, slots=True)
class PayloadAttachment:
    """把版本化 ``PayloadCollisionProfile`` 绑定到碰撞世界中的一个附着对象。

    规划世界里的载荷是**一个单刚体保守表示**。``CollisionGeometry`` 没有局部
    位姿，所以 Profile 的每个碰撞基元必须已经在载荷体坐标系对齐（``pose`` 为
    单位变换），并且只能是盒体或圆柱；球/锥、带局部位姿的基元、或只有
    ``collision_asset_ref`` 却没有基元的 Profile 一律**显式拒绝**，不用近似
    几何冒充载荷已经进入规划器。

    载荷体坐标系到法兰的关系由 ``AttachmentSpec.flange_from_body`` 显式给出
    （设备 provider 用实测抓取偏置推导），本类只负责校验它和碰撞世界一致。
    """

    attachment: AttachmentSpec
    profile: PayloadCollisionProfile

    def __post_init__(self) -> None:
        """校验载荷形状可表达、基元已对齐、附件类型正确。"""

        if not isinstance(self.attachment, AttachmentSpec):
            raise TypeError("PayloadAttachment.attachment 必须是 AttachmentSpec")
        if not isinstance(self.profile, PayloadCollisionProfile):
            raise TypeError("PayloadAttachment.profile 必须是 PayloadCollisionProfile")
        if not self.profile.collision_primitives:
            raise ValueError(
                "PayloadAttachment 必须由 PayloadCollisionProfile 的碰撞基元表达；"
                "只有 collision_asset_ref 时无法证明规划碰撞世界包含该载荷"
            )
        for index, primitive in enumerate(self.profile.collision_primitives):
            if primitive.shape not in ("box", "cylinder"):
                raise ValueError(
                    f"碰撞基元 {index} 的形状 {primitive.shape!r} 无法用规划 "
                    "CollisionGeometry 表达（只支持 box/cylinder）"
                )
            if not _is_identity_transform(
                primitive.pose.translation_m, primitive.pose.orientation_xyzw
            ):
                raise ValueError(
                    f"碰撞基元 {index} 必须已在载荷体坐标系对齐（pose 为单位变换）；"
                    "CollisionGeometry 没有局部位姿，带偏移的基元会被规划器静默错放"
                )

    @property
    def body_id(self) -> str:
        """返回载荷在碰撞世界/attachments 中的对象 id。"""

        return self.attachment.body_id

    @property
    def payload_ref(self) -> str:
        """返回载荷业务身份（Profile 的稳定引用）。"""

        return self.profile.profile_ref

    @property
    def profile_digest(self) -> str:
        """返回载荷 Profile 的 SHA-256。"""

        return self.profile.digest

    @property
    def profile_geometries(self) -> tuple[CollisionGeometry, ...]:
        """把 Profile 基元翻译成碰撞世界几何（圆柱按 MoveIt 的 length/radius）。"""

        geometries: list[CollisionGeometry] = []
        for primitive in self.profile.collision_primitives:
            if primitive.shape == "box":
                geometries.append(
                    CollisionGeometry("box", size_m=tuple(primitive.size_m))
                )
            else:
                length, radius = primitive.size_m
                geometries.append(
                    CollisionGeometry(
                        "cylinder", radius_m=radius, length_m=length, axis="z"
                    )
                )
        return tuple(geometries)

    def scene_problems(
        self, world: CollisionWorld, *, flange_link: str
    ) -> tuple[str, ...]:
        """返回载荷与碰撞世界不一致的问题清单；空元组表示可以规划。"""

        if not isinstance(world, CollisionWorld):
            raise TypeError("world 必须是 CollisionWorld")
        problems: list[str] = []
        if self.attachment.link_name != flange_link:
            problems.append(
                "payload 必须附着在法兰 "
                f"{flange_link}，实际 {self.attachment.link_name}"
            )
        target = next(
            (item for item in world.objects if item.object_id == self.body_id), None
        )
        if target is None:
            problems.append(f"碰撞世界不包含 payload 对象 {self.body_id}")
            return tuple(problems)
        if target.attached_to_link != self.attachment.link_name:
            problems.append(
                f"payload 对象 {self.body_id} 未标记为附着到 "
                f"{self.attachment.link_name}（attached_to_link="
                f"{target.attached_to_link!r}）"
            )
        missing = [
            geometry
            for geometry in self.profile_geometries
            if not _geometry_present(target.geometry, geometry)
        ]
        if missing:
            problems.append(
                f"碰撞世界中 {self.body_id} 的几何与 Profile 不一致，缺少 "
                f"{[item.as_mapping() for item in missing]}"
            )
        return tuple(problems)

    def as_mapping(self) -> dict[str, Any]:
        """返回载荷附件、载荷身份与完整规划 Profile 的规范映射。"""

        return {
            "body_id": self.body_id,
            "link_name": self.attachment.link_name,
            "flange_from_body": [
                list(row) for row in self.attachment.flange_from_body
            ],
            "touch_links": list(self.attachment.touch_links),
            "payload_ref": self.payload_ref,
            "profile_digest": self.profile_digest,
            "profile": self.profile.as_planning_scene(),
        }

    @property
    def digest(self) -> str:
        """返回载荷附件 + Profile 的稳定摘要。"""

        return _digest(self.as_mapping())


@dataclass(frozen=True, slots=True)
class PlanRequest:
    """一次无副作用规划请求。

    ``rail_axis`` 与 ``start_rail_m`` 必须同时出现或同时省略；导轨只作为
    独立轴，不并入六轴规划组。声明导轨时 ``goal_rail_m`` 默认等于
    ``start_rail_m``（该段保持导轨不动），后端必须把两个值都写进机器人
    状态；省略导轨轴时 ``goal_rail_m`` 不得单独出现。``options`` 只允许
    非语义性的后端调参，不得用它绕过关节限位、碰撞或本合同的校验。

    ``payload`` 是可选夹持载荷：声明后它必须已经在 ``collision_world`` 里、
    标记为附着到同一法兰、并出现在 ``attachments``/``carried_body_ids`` 中，
    否则请求在构造期失败。没有声明载荷时规范映射、摘要与 P17 之前逐位一致。
    """

    robot: RobotModel
    start_q: tuple[float, ...]
    goal: PlanGoal
    planning_group: str | None = None
    rail_axis: RailAxis | None = None
    start_rail_m: float | None = None
    goal_rail_m: float | None = None
    collision_world: CollisionWorld = field(default_factory=CollisionWorld)
    constraints: PlanConstraints = field(default_factory=PlanConstraints)
    seeds: tuple[tuple[float, ...], ...] = ()
    timeout_s: float = 10.0
    attachments: tuple[AttachmentSpec, ...] = ()
    carried_body_ids: tuple[str, ...] = ()
    held_material: str | None = None
    options: Mapping[str, Any] = field(default_factory=dict)
    payload: PayloadAttachment | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.robot, RobotModel):
            raise TypeError("robot 必须是 RobotModel")
        object.__setattr__(
            self, "start_q", self.robot.validate_joint_positions(self.start_q, name="start_q")
        )
        if not isinstance(self.goal, PlanGoal):
            raise TypeError("goal 必须是 PlanGoal")
        if self.goal.kind == "joint":
            assert self.goal.joint_positions is not None
            self.robot.validate_joint_positions(
                self.goal.joint_positions, name="goal.joint_positions"
            )
        else:
            assert self.goal.pose is not None
            if self.goal.pose.link_name is None:
                object.__setattr__(
                    self.goal,
                    "pose",
                    CartesianGoal(
                        self.goal.pose.frame_ref,
                        self.goal.pose.xyz_m,
                        self.goal.pose.orientation_xyzw,
                        self.robot.flange_link,
                        self.goal.pose.position_tolerance_m,
                        self.goal.pose.orientation_tolerance_rad,
                    ),
                )
        group = self.planning_group or self.robot.planning_group
        if group != self.robot.planning_group:
            raise ValueError("planning_group 必须与 RobotModel.planning_group 一致")
        object.__setattr__(self, "planning_group", group)
        if (self.rail_axis is None) != (self.start_rail_m is None):
            raise ValueError("rail_axis 与 start_rail_m 必须同时提供或同时省略")
        if self.rail_axis is None:
            if self.goal_rail_m is not None:
                raise ValueError("未声明 rail_axis 时不得提供 goal_rail_m")
        else:
            if not isinstance(self.rail_axis, RailAxis):
                raise TypeError("rail_axis 必须是 RailAxis")
            if self.rail_axis.axis_id in self.robot.joint_names:
                # 同名会让导轨坐标覆盖六轴状态里的一维，等于把独立轴并进规划组。
                raise ValueError("RailAxis.axis_id（导轨）不得与机械臂关节同名")
            if not self.rail_axis.contains(self.start_rail_m):
                raise ValueError("start_rail_m 超出导轨限位")
            if self.goal_rail_m is None:
                # 同一段内导轨保持不动时，终点导轨就是起点导轨；绝不回落到 0。
                object.__setattr__(self, "goal_rail_m", float(self.start_rail_m))
            if not self.rail_axis.contains(self.goal_rail_m):
                raise ValueError("goal_rail_m 超出导轨限位")
            object.__setattr__(
                self,
                "start_rail_m",
                _finite(self.start_rail_m, "start_rail_m"),
            )
            object.__setattr__(
                self,
                "goal_rail_m",
                _finite(self.goal_rail_m, "goal_rail_m"),
            )
        if not isinstance(self.collision_world, CollisionWorld):
            raise TypeError("collision_world 必须是 CollisionWorld")
        if not isinstance(self.constraints, PlanConstraints):
            raise TypeError("constraints 必须是 PlanConstraints")
        safe = self.constraints.safe_pose
        if safe is not None:
            self.robot.validate_joint_positions(
                safe.joint_positions, name="safe_pose.joint_positions"
            )
        if isinstance(self.seeds, (str, bytes)):
            raise TypeError("seeds 必须是关节序列的序列")
        seeds = tuple(
            self.robot.validate_joint_positions(seed, name=f"seeds[{index}]")
            for index, seed in enumerate(self.seeds)
        )
        object.__setattr__(self, "seeds", seeds)
        object.__setattr__(
            self, "timeout_s", _finite(self.timeout_s, "timeout_s", minimum=1e-6)
        )
        if self.timeout_s > _MAX_TIMEOUT_S:
            raise ValueError(f"timeout_s 不得超过 {_MAX_TIMEOUT_S}")
        if isinstance(self.attachments, (str, bytes)):
            raise TypeError("attachments 必须是 AttachmentSpec 序列")
        attachments = tuple(self.attachments)
        if any(not isinstance(item, AttachmentSpec) for item in attachments):
            raise TypeError("attachments 只能包含 AttachmentSpec")
        attached_ids = [item.body_id for item in attachments]
        if len(set(attached_ids)) != len(attached_ids):
            raise ValueError("attachments body_id 不得重复")
        unknown = set(attached_ids) - set(self.collision_world.object_ids)
        if unknown:
            raise ValueError(f"attachments 引用了碰撞世界不存在的对象: {sorted(unknown)}")
        declared_attached = {
            item.object_id
            for item in self.collision_world.objects
            if item.attached_to_link is not None
        }
        if declared_attached - set(attached_ids):
            raise ValueError(
                "碰撞世界中标记为附着的对象必须出现在 attachments: "
                f"{sorted(declared_attached - set(attached_ids))}"
            )
        attachment_by_id = {item.body_id: item for item in attachments}
        for attachment in attachments:
            # 工具/物料仍挂法兰；滑座等装配件必须显式声明为 robot_fixture。
            if attachment.purpose == "tool_or_payload" and attachment.link_name != self.robot.flange_link:
                raise ValueError("attachment.link_name 必须是法兰 link")
            if attachment.purpose == "robot_fixture":
                links={link.get("name") for link in ET.fromstring(self.robot.urdf_xml).findall("link")}
                if attachment.link_name not in links:
                    raise ValueError("robot_fixture attachment link absent from URDF")
        for obj in self.collision_world.objects:
            if obj.attached_to_link is None:
                continue
            spec = attachment_by_id[obj.object_id]
            if spec.link_name != obj.attached_to_link:
                raise ValueError(
                    f"对象 {obj.object_id} 的 attached_to_link 与 AttachmentSpec 不一致"
                )
        object.__setattr__(self, "attachments", attachments)
        if isinstance(self.carried_body_ids, (str, bytes)):
            raise TypeError("carried_body_ids 必须是对象 id 序列")
        carried = tuple(str(item).strip() for item in self.carried_body_ids)
        if any(not item for item in carried) or len(set(carried)) != len(carried):
            raise ValueError("carried_body_ids 不得重复或包含空字符串")
        if set(carried) - set(attached_ids):
            raise ValueError("carried_body_ids 必须全部出现在 attachments 中")
        if any(attachment_by_id[key].purpose=="robot_fixture" for key in carried):
            raise ValueError("robot_fixture must not be declared as carried payload")
        object.__setattr__(self, "carried_body_ids", carried)
        if self.held_material is not None:
            object.__setattr__(
                self, "held_material", _text(self.held_material, "held_material")
            )
            if not carried:
                raise ValueError("held_material 存在时 carried_body_ids 不得为空")
        if self.payload is not None:
            if not isinstance(self.payload, PayloadAttachment):
                raise TypeError("payload 必须是 PayloadAttachment 或 None")
            declared = attachment_by_id.get(self.payload.body_id)
            if declared is None:
                raise ValueError(
                    "payload.attachment 必须出现在 attachments 中（body_id="
                    f"{self.payload.body_id}）"
                )
            if declared != self.payload.attachment:
                raise ValueError(
                    f"payload.attachment 与 attachments 中 {self.payload.body_id} "
                    "的声明不一致"
                )
            if self.payload.body_id not in carried:
                raise ValueError("payload 必须出现在 carried_body_ids 中")
            if (
                self.held_material is not None
                and self.held_material != self.payload.payload_ref
            ):
                raise ValueError(
                    "held_material 与 payload identity 不一致："
                    f"{self.held_material} != {self.payload.payload_ref}"
                )
            problems = self.payload.scene_problems(
                self.collision_world, flange_link=self.robot.flange_link
            )
            if problems:
                raise ValueError("；".join(problems))
        object.__setattr__(self, "options", _mapping(self.options, "options"))

    @property
    def start_rail(self) -> float | None:
        """``start_rail_m`` 的短别名；单位仍是米。"""

        return self.start_rail_m

    @property
    def goal_rail(self) -> float | None:
        """``goal_rail_m`` 的短别名；声明导轨时总等于一个显式值。"""

        return self.goal_rail_m

    @property
    def rail_moves(self) -> bool:
        """返回本请求是否要求导轨在段内移动。"""

        if self.start_rail_m is None or self.goal_rail_m is None:
            return False
        return abs(float(self.goal_rail_m) - float(self.start_rail_m)) > 1e-9

    @property
    def payload_fields(self) -> dict[str, str]:
        """返回 ``PlanResult`` 回显载荷身份所需的三个字段；无载荷时为空。"""

        if self.payload is None:
            return {}
        return {
            "payload_ref": self.payload.payload_ref,
            "payload_profile_digest": self.payload.profile_digest,
            "attachment_digest": self.payload.digest,
        }

    @property
    def digest(self) -> str:
        """返回请求的稳定摘要，不包含本机路径或时间戳。"""

        return _digest(self.as_mapping())

    def as_mapping(self) -> dict[str, Any]:
        """返回请求的规范表示。"""

        result: dict[str, Any] = {
            "schema": _SCHEMA,
            "robot": self.robot.as_mapping(),
            "start_q": list(self.start_q),
            "goal": self.goal.as_mapping(),
            "planning_group": self.planning_group,
            "rail_axis": self.rail_axis.as_mapping() if self.rail_axis else None,
            "start_rail_m": self.start_rail_m,
            "goal_rail_m": self.goal_rail_m,
            "collision_world": self.collision_world.as_mapping(),
            "constraints": self.constraints.as_mapping(),
            "seeds": [list(seed) for seed in self.seeds],
            "timeout_s": self.timeout_s,
            "attachments": [
                {
                    "body_id": item.body_id,
                    "link_name": item.link_name,
                    "flange_from_body": [list(row) for row in item.flange_from_body],
                    "touch_links": list(item.touch_links),
                    **({"purpose":item.purpose} if item.purpose!="tool_or_payload" else {}),
                }
                for item in self.attachments
            ],
            "carried_body_ids": list(self.carried_body_ids),
            "held_material": self.held_material,
            "options": self.options,
        }
        if self.payload is not None:
            # 只有真正声明载荷时才出现该键：无载荷请求的规范映射/摘要逐位不变。
            result["payload"] = {
                "payload_ref": self.payload.payload_ref,
                "profile_digest": self.payload.profile_digest,
                "attachment_digest": self.payload.digest,
                "body_id": self.payload.body_id,
                "link_name": self.payload.attachment.link_name,
            }
        return result


@dataclass(frozen=True, slots=True)
class PlanWaypoint:
    """规划轨迹上的单个定时状态。"""

    q: tuple[float, ...]
    rail_m: float
    time_s: float
    source: WaypointSource = "planner"
    velocities: tuple[float, ...] | None = None
    accelerations: tuple[float, ...] | None = None
    rail_velocity: float | None = None
    rail_acceleration: float | None = None

    def __post_init__(self) -> None:
        if isinstance(self.q, (str, bytes)):
            raise TypeError("waypoint.q 必须是数值序列")
        values = tuple(_finite(item, f"waypoint.q[{index}]") for index, item in enumerate(self.q))
        if not values:
            raise ValueError("waypoint.q 必须是非空有限数值序列")
        object.__setattr__(self, "q", values)
        object.__setattr__(self, "rail_m", _finite(self.rail_m, "rail_m"))
        object.__setattr__(
            self, "time_s", _finite(self.time_s, "time_s", minimum=0.0)
        )
        if not isinstance(self.source, str) or not self.source.strip():
            raise ValueError("waypoint.source 必须是非空字符串")
        for name in ("velocities", "accelerations"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _float_tuple(value, len(values), name))
        for name in ("rail_velocity", "rail_acceleration"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, name))

    def as_mapping(self) -> dict[str, Any]:
        """返回全部原生路点字段；旧的无导数路点摘要保持不变。"""
        result = {
            "q": list(self.q),
            "rail_m": self.rail_m,
            "time_s": self.time_s,
            "source": self.source,
        }
        for name in ("velocities", "accelerations", "rail_velocity", "rail_acceleration"):
            value = getattr(self, name)
            if value is not None:
                result[name] = list(value) if isinstance(value, tuple) else value
        return result


@dataclass(frozen=True, slots=True)
class PlannerIdentity:
    """规划器身份与真实探测到的引擎版本。"""

    name: str
    implementation: str
    version: str
    engine_versions: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _text(self.name, "name"))
        object.__setattr__(
            self, "implementation", _text(self.implementation, "implementation")
        )
        object.__setattr__(self, "version", _text(self.version, "version"))
        versions = _mapping(self.engine_versions, "engine_versions")
        object.__setattr__(
            self,
            "engine_versions",
            {str(key): str(value) for key, value in versions.items()},
        )

    def as_mapping(self) -> dict[str, Any]:
        """返回摘要使用的身份字段。"""

        return {
            "name": self.name,
            "implementation": self.implementation,
            "version": self.version,
            "engine_versions": self.engine_versions,
        }


@dataclass(frozen=True, slots=True)
class PlanResult:
    """规划器返回的不可变结果。

    成功结果必须包含至少一个路点；失败结果可以带部分路点（例如已规划
    的安全段），但调用方不得把非 ``success`` 结果当作可执行轨迹。

    ``payload_ref``/``payload_profile_digest``/``attachment_digest`` 是规划器
    对“本次结果是在携带该载荷的场景里算出”的回显：三者必须同时出现或同时
    省略。没有携带载荷的请求，其结果不得携带这三个字段（``validate_payload``
    会拒绝），因此旧结果/旧摘要不受影响。
    """

    status: PlanStatus
    waypoints: tuple[PlanWaypoint, ...]
    planner: PlannerIdentity
    collision_checks: int = 0
    safe_pose_used: bool = False
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    digest: str = ""
    payload_ref: str | None = None
    payload_profile_digest: str | None = None
    attachment_digest: str | None = None

    def __post_init__(self) -> None:
        if self.status not in PLAN_STATUSES:
            raise ValueError(f"未知 PlanResult.status: {self.status}")
        if isinstance(self.waypoints, (str, bytes)):
            raise TypeError("waypoints 必须是 PlanWaypoint 序列")
        waypoints = tuple(self.waypoints)
        if any(not isinstance(item, PlanWaypoint) for item in waypoints):
            raise TypeError("waypoints 只能包含 PlanWaypoint")
        if self.status == "success" and not waypoints:
            raise ValueError("成功规划结果不得包含空轨迹")
        if waypoints:
            dimensions = {len(item.q) for item in waypoints}
            if len(dimensions) != 1:
                raise ValueError("所有路点必须具有相同关节维度")
            times = [item.time_s for item in waypoints]
            if any(later < earlier for earlier, later in zip(times, times[1:])):
                raise ValueError("waypoint.time_s 必须非递减")
        if not isinstance(self.planner, PlannerIdentity):
            raise TypeError("planner 必须是 PlannerIdentity")
        if isinstance(self.collision_checks, bool) or self.collision_checks < 0:
            raise ValueError("collision_checks 必须是非负整数")
        object.__setattr__(self, "waypoints", waypoints)
        object.__setattr__(self, "collision_checks", int(self.collision_checks))
        declared = (
            self.payload_ref,
            self.payload_profile_digest,
            self.attachment_digest,
        )
        if any(value is not None for value in declared):
            if any(value is None for value in declared):
                raise ValueError(
                    "PlanResult 的 payload_ref/payload_profile_digest/"
                    "attachment_digest 必须同时提供或同时省略"
                )
            object.__setattr__(
                self, "payload_ref", _text(self.payload_ref, "payload_ref")
            )
            for name, value in (
                ("payload_profile_digest", self.payload_profile_digest),
                ("attachment_digest", self.attachment_digest),
            ):
                if not isinstance(value, str) or not _SHA256.fullmatch(value):
                    raise ValueError(f"PlanResult.{name} 必须是 SHA-256")
        diagnostics = _mapping(self.diagnostics, "diagnostics")
        object.__setattr__(self, "diagnostics", diagnostics)
        computed = _digest(self.as_mapping(include_digest=False))
        if self.digest and self.digest != computed:
            raise ValueError("PlanResult.digest 与内容不一致")
        object.__setattr__(self, "digest", computed)

    @property
    def planner_identity(self) -> PlannerIdentity:
        """``planner`` 的显式别名，便于消费方表达版本身份。"""

        return self.planner

    def as_mapping(self, *, include_digest: bool = True) -> dict[str, Any]:
        """返回结果规范表示；摘要计算时排除自身。"""

        result: dict[str, Any] = {
            "status": self.status,
            "waypoints": [item.as_mapping() for item in self.waypoints],
            "planner": self.planner.as_mapping(),
            "collision_checks": self.collision_checks,
            "safe_pose_used": self.safe_pose_used,
            "diagnostics": self.diagnostics,
        }
        if self.payload_ref is not None:
            result["payload"] = {
                "payload_ref": self.payload_ref,
                "payload_profile_digest": self.payload_profile_digest,
                "attachment_digest": self.attachment_digest,
            }
        if include_digest:
            result["digest"] = self.digest
        return result

    def with_payload(
        self, request: PlanRequest, **overrides: str
    ) -> PlanResult:
        """回显请求携带的载荷身份；无载荷时返回自身。

        规划后端在返回结果前调用它：只有真正把载荷装进自己规划场景的后端
        才会做这一步。协调器把“没有回显”当成规划器没有证明载荷进入场景，
        整个计划失败关闭。``overrides`` 只用于测试/诊断替换某个字段，不得
        与请求身份不一致（否则仍然会在 ``validate_payload`` 处被拒绝）。
        """

        if not isinstance(request, PlanRequest):
            raise TypeError("request 必须是 PlanRequest")
        fields = {**request.payload_fields, **overrides}
        if not fields:
            if self.payload_ref is not None:
                raise ValueError("结果携带了请求未声明的 payload 身份")
            return self
        current = {
            "payload_ref": self.payload_ref,
            "payload_profile_digest": self.payload_profile_digest,
            "attachment_digest": self.attachment_digest,
        }
        if any(value is not None for value in current.values()):
            if current != fields:
                raise ValueError("结果已记录与请求不一致的 payload 身份")
            return self
        return PlanResult(
            status=self.status,
            waypoints=self.waypoints,
            planner=self.planner,
            collision_checks=self.collision_checks,
            safe_pose_used=self.safe_pose_used,
            diagnostics=self.diagnostics,
            payload_ref=fields["payload_ref"],
            payload_profile_digest=fields["payload_profile_digest"],
            attachment_digest=fields["attachment_digest"],
        )

    def validate_payload(self, request: PlanRequest) -> PlanResult:
        """核对结果回显的载荷身份/Profile/附件摘要是否与请求一致。"""

        if not isinstance(request, PlanRequest):
            raise TypeError("request 必须是 PlanRequest")
        expected = request.payload
        has_echo = self.payload_ref is not None
        if expected is None:
            if has_echo:
                raise ValueError(
                    "结果携带了请求未声明的 payload 身份: " f"{self.payload_ref}"
                )
            return self
        if not has_echo:
            raise ValueError(
                "结果未记录请求携带的 payload identity/profile/attachment digest"
            )
        if self.payload_ref != expected.payload_ref:
            raise ValueError(
                "payload identity 不一致: "
                f"{self.payload_ref} != {expected.payload_ref}"
            )
        if self.payload_profile_digest != expected.profile_digest:
            raise ValueError("payload profile digest 与请求不一致")
        if self.attachment_digest != expected.digest:
            raise ValueError("attachment digest 与请求不一致")
        return self

    def validate_against(self, request: PlanRequest) -> PlanResult:
        """按原请求复核载荷身份、维度、导轨限位和关节限位。

        这里不重新做碰撞查询；它用于阻止把别的机器人/载荷/导轨/维度结果
        误配给当前请求，或把越界值送入执行器。
        """

        if not isinstance(request, PlanRequest):
            raise TypeError("request 必须是 PlanRequest")
        self.validate_payload(request)
        for index, waypoint in enumerate(self.waypoints):
            request.robot.validate_joint_positions(
                waypoint.q, name=f"waypoints[{index}].q"
            )
            if request.rail_axis is not None and not request.rail_axis.contains(
                waypoint.rail_m
            ):
                raise ValueError(f"waypoints[{index}].rail_m 超出导轨限位")
            if request.rail_axis is None and abs(waypoint.rail_m) > 1e-9:
                # 未声明导轨轴时 0 是唯一约定值；非零说明结果来自别的请求。
                raise ValueError(
                    f"waypoints[{index}].rail_m 必须为 0（请求未声明导轨轴）"
                )
        if self.waypoints and request.rail_axis is not None:
            rail_tolerance = float(request.options.get("rail_goal_tolerance_m", 1e-4))
            first = self.waypoints[0]
            last = self.waypoints[-1]
            assert request.start_rail_m is not None
            assert request.goal_rail_m is not None
            if abs(first.rail_m - request.start_rail_m) > 1e-6:
                raise ValueError("轨迹首点导轨与请求 start_rail_m 不一致")
            if abs(last.rail_m - request.goal_rail_m) > rail_tolerance:
                raise ValueError("轨迹末点导轨与请求 goal_rail_m 不一致")
        if self.waypoints:
            first = self.waypoints[0]
            if max(
                abs(left - right)
                for left, right in zip(first.q, request.start_q, strict=True)
            ) > 1e-6:
                raise ValueError("轨迹首点与请求 start_q 不一致")
            if request.goal.kind == "joint":
                assert request.goal.joint_positions is not None
                tolerance = float(request.options.get("goal_tolerance_rad", 1e-3))
                if max(
                    abs(left - right)
                    for left, right in zip(
                        self.waypoints[-1].q, request.goal.joint_positions, strict=True
                    )
                ) > tolerance:
                    raise ValueError("轨迹末点与关节目标不一致")
        return self


@dataclass(frozen=True, slots=True)
class JointContinuityLimits:
    """相邻段端点与段内相邻路点的关节连续性硬阈值（弧度）。

    ``max_segment_gap_rad`` 默认 1e-6：上一段末路点与下一段首路点必须逐位
    相接；``two_pi_tolerance_rad`` 决定“离 2π 整数倍多近”算一次整圈重标定。
    两个阈值都只允许有限非负值，不允许用负值放行不连续。
    """

    max_segment_gap_rad: float = 1e-6
    two_pi_tolerance_rad: float = 1e-3

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_segment_gap_rad",
            _finite(self.max_segment_gap_rad, "max_segment_gap_rad", minimum=0.0),
        )
        object.__setattr__(
            self,
            "two_pi_tolerance_rad",
            _finite(
                self.two_pi_tolerance_rad, "two_pi_tolerance_rad", minimum=0.0
            ),
        )

    def as_mapping(self) -> dict[str, float]:
        """返回报告用阈值字段。"""

        return {
            "max_segment_gap_rad": self.max_segment_gap_rad,
            "two_pi_tolerance_rad": self.two_pi_tolerance_rad,
        }


def _whole_turns(gap_rad: float, tolerance_rad: float) -> int:
    """返回 ``gap_rad`` 等价于几个整圈；不足一圈或不在容差内返回 0。"""

    if gap_rad <= 0.0:
        return 0
    turns = int(round(gap_rad / _TWO_PI))
    if turns >= 1 and abs(gap_rad - turns * _TWO_PI) <= tolerance_rad:
        return turns
    return 0


@dataclass(frozen=True, slots=True)
class JointContinuityReport:
    """一条多段轨迹链的关节连续性报告。"""

    limits: JointContinuityLimits
    segment_count: int
    max_segment_gap_rad: float
    violations: tuple[Mapping[str, Any], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.limits, JointContinuityLimits):
            raise TypeError("limits 必须是 JointContinuityLimits")
        if isinstance(self.segment_count, bool) or self.segment_count < 0:
            raise ValueError("segment_count 必须是非负整数")
        object.__setattr__(
            self,
            "max_segment_gap_rad",
            _finite(self.max_segment_gap_rad, "max_segment_gap_rad", minimum=0.0),
        )
        violations = tuple(
            _mapping(item, f"violations[{index}]")
            for index, item in enumerate(self.violations)
        )
        object.__setattr__(self, "violations", violations)

    @property
    def ok(self) -> bool:
        """返回整条链是否关节连续。"""

        return not self.violations

    def as_mapping(self) -> dict[str, Any]:
        """返回写进诊断/报告的规范映射。"""

        return {
            "ok": self.ok,
            "segment_count": self.segment_count,
            "max_segment_gap_rad": self.max_segment_gap_rad,
            "limits": self.limits.as_mapping(),
            "violations": [dict(item) for item in self.violations],
        }


def check_joint_continuity(
    segments: Sequence[PlanResult],
    *,
    limits: JointContinuityLimits | None = None,
) -> JointContinuityReport:
    """检查一条 ``PlanResult`` 链的关节连续性。

    参数：``segments`` 是按执行顺序排列的规划结果；``limits`` 缺省用合同
    最严阈值。返回：``JointContinuityReport``；``ok`` 为假表示这条链不能当成
    一条可执行轨迹。异常：类型错误。

    检查两类事实：

    1. 相邻段端点：上一段末路点与下一段首路点的逐轴 ``|Δq|`` 必须
       ≤ ``max_segment_gap_rad``，且落在 2π 整数倍容差内的差值单独记为
       ``two_pi_jump``（整圈重标定，动力学臂无法直接跟随）；
    2. 段内相邻路点：任何单步 2π 整数倍跳变同样是 ``two_pi_jump``。

    没有路点的段记为 ``missing_trajectory``，不静默跳过。
    """

    if limits is None:
        limits = JointContinuityLimits()
    if not isinstance(limits, JointContinuityLimits):
        raise TypeError("limits 必须是 JointContinuityLimits")
    if isinstance(segments, (str, bytes)):
        raise TypeError("segments 必须是 PlanResult 序列")
    items = tuple(segments)
    if any(not isinstance(item, PlanResult) for item in items):
        raise TypeError("segments 只能包含 PlanResult")

    violations: list[dict[str, Any]] = []
    max_gap = 0.0
    previous: PlanResult | None = None
    for index, item in enumerate(items):
        if not item.waypoints:
            violations.append(
                {
                    "kind": "missing_trajectory",
                    "segment_index": index,
                    "status": item.status,
                }
            )
            previous = None
            continue
        for waypoint_index in range(1, len(item.waypoints)):
            before = item.waypoints[waypoint_index - 1].q
            after = item.waypoints[waypoint_index].q
            for joint_index, (left, right) in enumerate(
                zip(before, after, strict=True)
            ):
                gap = abs(right - left)
                turns = _whole_turns(gap, limits.two_pi_tolerance_rad)
                if turns:
                    violations.append(
                        {
                            "kind": "two_pi_jump",
                            "scope": "waypoint",
                            "segment_index": index,
                            "waypoint_index": waypoint_index,
                            "joint_index": joint_index,
                            "turns": turns,
                            "gap_rad": gap,
                        }
                    )
        if previous is not None:
            left_q = previous.waypoints[-1].q
            right_q = item.waypoints[0].q
            if len(left_q) != len(right_q):
                violations.append(
                    {
                        "kind": "dimension_mismatch",
                        "scope": "segment",
                        "segment_index": index,
                        "previous_dimension": len(left_q),
                        "dimension": len(right_q),
                    }
                )
            else:
                gaps = [
                    abs(float(left) - float(right))
                    for left, right in zip(left_q, right_q, strict=True)
                ]
                gap = max(gaps) if gaps else 0.0
                max_gap = max(max_gap, gap)
                if gap > limits.max_segment_gap_rad:
                    joint_index = gaps.index(gap)
                    turns = _whole_turns(gap, limits.two_pi_tolerance_rad)
                    violation: dict[str, Any] = {
                        "kind": "two_pi_jump" if turns else "segment_gap",
                        "scope": "segment",
                        "segment_index": index,
                        "joint_index": joint_index,
                        "gap_rad": gap,
                    }
                    if turns:
                        violation["turns"] = turns
                    violations.append(violation)
        previous = item
    return JointContinuityReport(
        limits=limits,
        segment_count=len(items),
        max_segment_gap_rad=max_gap,
        violations=tuple(violations),
    )


@runtime_checkable
class MotionPlanner(Protocol):
    """所有规划后端的唯一公共入口。"""

    def plan(self, request: PlanRequest) -> PlanResult:
        """规划请求；不得执行硬件动作，也不得返回空成功轨迹。"""


@runtime_checkable
class SceneValidatingPlanner(Protocol):
    """可选场景校验能力；不与 ``plan`` 的返回类型混用。"""

    def validate(self, scene: CollisionWorld) -> Mapping[str, Any]:
        """校验场景值对象；不声称已经完成碰撞自由证明。"""


__all__ = [
    "PLAN_STATUSES",
    "AttachmentSpec",
    "CartesianGoal",
    "ClearanceConstraint",
    "CollisionGeometry",
    "CollisionObject",
    "CollisionWorld",
    "JointContinuityLimits",
    "JointContinuityReport",
    "MotionPlanner",
    "OrientationConstraintSpec",
    "PayloadAttachment",
    "PlanConstraints",
    "PlanGoal",
    "PlanRequest",
    "PlanResult",
    "PlanStatus",
    "PlanWaypoint",
    "PlannerIdentity",
    "RailAxis",
    "RobotModel",
    "SafePoseConstraint",
    "SceneValidatingPlanner",
    "WaypointSource",
    "check_joint_continuity",
]
