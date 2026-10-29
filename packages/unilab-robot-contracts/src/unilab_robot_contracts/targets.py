"""把规范机械臂目标解析为执行 Adapter 可接受的绝对目标。"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol, TypeAlias

from .geometry import RigidTransform


class JointType(str, Enum):
    """决定关节值规范 SI 单位的机械关节类型。"""

    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"

    @property
    def canonical_unit(self) -> str:
        """返回该关节类型唯一允许的 SI 单位。"""

        return "m" if self is JointType.PRISMATIC else "rad"


@dataclass(frozen=True, slots=True)
class JointSpecification:
    """由精确机械臂型号拥有的关节顺序、类型和 SI 限位。"""

    name: str
    joint_type: JointType
    lower: float | None
    upper: float | None

    def __post_init__(self) -> None:
        """校验稳定名称和上下限顺序。"""

        if not self.name.strip():
            raise ValueError("JointSpecification.name 不能为空")
        if (self.lower is None) != (self.upper is None):
            raise ValueError(f"关节 {self.name} 必须同时声明上下限")
        if self.lower is not None and self.upper is not None:
            if not math.isfinite(self.lower) or not math.isfinite(self.upper):
                raise ValueError(f"关节 {self.name} 限位必须是有限数")
            if self.lower > self.upper:
                raise ValueError(f"关节 {self.name} 下限不得大于上限")


@dataclass(frozen=True, slots=True)
class CartesianPose:
    """一个在明确坐标系中表达的绝对 TCP 位姿。"""

    frame_ref: str
    xyz_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        """校验坐标系，并用刚体值对象规范化数值。"""

        if not self.frame_ref.strip():
            raise ValueError("CartesianPose.frame_ref 不能为空")
        transform = RigidTransform(self.xyz_m, self.orientation_xyzw)
        object.__setattr__(self, "xyz_m", transform.translation_m)
        object.__setattr__(self, "orientation_xyzw", transform.orientation_xyzw)

    @property
    def transform(self) -> RigidTransform:
        """返回与该绝对位姿等价的刚体变换。"""

        return RigidTransform(self.xyz_m, self.orientation_xyzw)


@dataclass(frozen=True, slots=True)
class ToolContext:
    """一次点位解析冻结的工具实例、TCP 变换和附着代次。"""

    context_id: str
    digest: str
    mount_to_tcp: RigidTransform
    attachment_generation: int
    planning_scene: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """校验工具身份、64 位摘要和正附着代次。"""

        if not self.context_id.strip():
            raise ValueError("ToolContext.context_id 不能为空")
        if len(self.digest) != 64 or any(
            character not in "0123456789abcdef" for character in self.digest.lower()
        ):
            raise ValueError("ToolContext.digest 必须是 64 位 SHA-256")
        if (
            isinstance(self.attachment_generation, bool)
            or self.attachment_generation < 1
        ):
            raise ValueError("ToolContext.attachment_generation 必须是正整数")
        if not isinstance(self.planning_scene, Mapping):
            raise TypeError("ToolContext.planning_scene 必须是对象")


class ArmTargetModel(Protocol):
    """点位解析器所需的最小型号运动学接口。"""

    model_ref: str
    base_frame: str
    joint_specs: tuple[JointSpecification, ...]

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        tool_context: ToolContext,
    ) -> CartesianPose:
        """把型号顺序的关节 SI 值转换为绝对 TCP 位姿。"""


@dataclass(frozen=True, slots=True)
class ResolvedJointTarget:
    """通过型号顺序和限位验证的绝对关节目标。"""

    target_ref: str
    joint_positions: tuple[float, ...]
    source_chain: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ResolvedCartesianTarget:
    """已展开全部相对依赖并转换到机械臂基座的绝对 TCP 目标。"""

    target_ref: str
    pose: CartesianPose
    source_chain: tuple[str, ...]
    ik_seed: tuple[float, ...] | None


ResolvedMotionTarget: TypeAlias = ResolvedJointTarget | ResolvedCartesianTarget


class ArmTargetResolver:
    """隐藏关节限位、FK、静态坐标变换和相对引用的共享深模块。"""

    def __init__(
        self,
        targets: Mapping[str, Mapping[str, Any]],
        *,
        model: ArmTargetModel,
        tool_context: ToolContext,
        frame_transforms: Mapping[str, RigidTransform] | None = None,
        max_reference_depth: int = 16,
        max_delta_translation_m: float = 1.0,
    ) -> None:
        """冻结规范目标、精确型号、工具上下文和静态坐标变换。

        参数：``targets`` 使用稳定 target ref；``frame_transforms`` 把设备局部
        坐标转换到型号基座坐标。返回：无。异常：目标、限位或变换不合法时
        抛出 ``ValueError``。本类不识别 PointSet Schema、库位（Site）或导轨。
        """

        if isinstance(max_reference_depth, bool) or max_reference_depth < 1:
            raise ValueError("max_reference_depth 必须是正整数")
        if not math.isfinite(max_delta_translation_m) or max_delta_translation_m <= 0.0:
            raise ValueError("max_delta_translation_m 必须是正有限数")
        self.model = model
        self.tool_context = tool_context
        self.max_reference_depth = max_reference_depth
        self.max_delta_translation_m = max_delta_translation_m
        self._targets = {
            str(target_ref): _mapping(target, str(target_ref))
            for target_ref, target in targets.items()
        }
        if not self._targets:
            raise ValueError("机械臂目标集合不能为空")
        if any("unit" in target for target in self._targets.values()):
            raise ValueError("目标不得重复 unit；单位由精确型号和字段名拥有")
        transforms = dict(frame_transforms or {})
        transforms.setdefault(model.base_frame, RigidTransform.identity())
        transforms.setdefault("arm_base", RigidTransform.identity())
        self._frame_transforms = transforms
        self._resolved: dict[str, ResolvedMotionTarget] = {}

    def resolve(self, target_ref: str) -> ResolvedMotionTarget:
        """把一个稳定引用解析为关节目标或基座坐标系绝对 TCP 目标。

        参数：``target_ref`` 是调用者拥有的稳定引用。返回：不可变解析目标。
        异常：引用缺失、成环、越界或坐标变换缺失时抛出 ``ValueError``。
        """

        return self._resolve(str(target_ref), visiting=())

    def resolve_all(self) -> Mapping[str, ResolvedMotionTarget]:
        """解析全部目标，使坏引用在激活而非物理派发阶段失败。"""

        for target_ref in self._targets:
            self.resolve(target_ref)
        return dict(self._resolved)

    def _resolve(
        self,
        target_ref: str,
        *,
        visiting: tuple[str, ...],
    ) -> ResolvedMotionTarget:
        """递归解析一个目标，并在当前调用链中检测循环。"""

        cached = self._resolved.get(target_ref)
        if cached is not None:
            return cached
        if target_ref in visiting:
            cycle = " -> ".join((*visiting, target_ref))
            raise ValueError(f"cartesian_delta relative_to 存在循环: {cycle}")
        if len(visiting) >= self.max_reference_depth:
            raise ValueError("cartesian_delta relative_to 超过最大引用深度")
        try:
            raw = self._targets[target_ref]
        except KeyError as exc:
            raise ValueError(f"目标集合缺少 target_ref: {target_ref}") from exc
        target_type = str(raw.get("type", ""))
        if target_type == "joint_positions":
            resolved = self._resolve_joint(target_ref, raw)
        elif target_type == "cartesian_pose":
            resolved = self._resolve_pose(target_ref, raw)
        elif target_type == "cartesian_delta":
            resolved = self._resolve_delta(
                target_ref,
                raw,
                visiting=(*visiting, target_ref),
            )
        else:
            raise ValueError(f"{target_ref} 使用未知 target type: {target_type}")
        self._resolved[target_ref] = resolved
        return resolved

    def _resolve_joint(
        self,
        target_ref: str,
        raw: Mapping[str, Any],
    ) -> ResolvedJointTarget:
        """按型号拥有的关节类型、顺序和 SI 限位解析数组。"""

        values = numeric_sequence(raw.get("value"), "joint_positions.value")
        if len(values) != len(self.model.joint_specs):
            raise ValueError(
                f"{target_ref} 关节数量必须为 {len(self.model.joint_specs)}"
            )
        for specification, value in zip(self.model.joint_specs, values, strict=True):
            if specification.lower is not None and value < specification.lower:
                raise ValueError(
                    f"{target_ref} 关节 {specification.name} 低于 "
                    f"{specification.lower}{specification.joint_type.canonical_unit}"
                )
            if specification.upper is not None and value > specification.upper:
                raise ValueError(
                    f"{target_ref} 关节 {specification.name} 高于 "
                    f"{specification.upper}{specification.joint_type.canonical_unit}"
                )
        return ResolvedJointTarget(target_ref, values, (target_ref,))

    def _resolve_pose(
        self,
        target_ref: str,
        raw: Mapping[str, Any],
    ) -> ResolvedCartesianTarget:
        """把明确来源坐标系中的绝对 TCP 位姿转换到机械臂基座。"""

        frame_ref = str(raw.get("frame_ref", ""))
        source_to_base = self._frame_transform(target_ref, frame_ref)
        local_pose = transform_value(raw.get("value"), target_ref)
        goal = source_to_base.compose(local_pose)
        return ResolvedCartesianTarget(
            target_ref,
            CartesianPose(
                self.model.base_frame,
                goal.translation_m,
                goal.orientation_xyzw,
            ),
            (target_ref,),
            None,
        )

    def _resolve_delta(
        self,
        target_ref: str,
        raw: Mapping[str, Any],
        *,
        visiting: tuple[str, ...],
    ) -> ResolvedCartesianTarget:
        """解析基准目标，并在明确坐标系中应用一段相对变换。"""

        relative_to = str(raw.get("relative_to", "")).strip()
        if not relative_to:
            raise ValueError(f"{target_ref} cartesian_delta 缺少 relative_to")
        base = self._resolve(relative_to, visiting=visiting)
        if isinstance(base, ResolvedJointTarget):
            base_pose = self.model.forward_kinematics(
                base.joint_positions,
                self.tool_context,
            )
            if base_pose.frame_ref != self.model.base_frame:
                raise ValueError(f"{relative_to} FK 没有返回型号基座坐标系")
            source_chain = base.source_chain
            ik_seed = base.joint_positions
        else:
            base_pose = base.pose
            source_chain = base.source_chain
            ik_seed = base.ik_seed
        delta = transform_value(raw.get("value"), target_ref)
        magnitude = math.sqrt(sum(value * value for value in delta.translation_m))
        if magnitude > self.max_delta_translation_m:
            raise ValueError(
                f"{target_ref} 平移偏移 {magnitude}m 超过上限 "
                f"{self.max_delta_translation_m}m"
            )
        frame_ref = str(raw.get("frame_ref", ""))
        if frame_ref == "reference_target":
            goal = base_pose.transform.compose(delta)
        else:
            frame_to_base = self._frame_transform(target_ref, frame_ref)
            rotated_translation = frame_to_base.rotate_vector(delta.translation_m)
            translated = tuple(
                base_pose.xyz_m[index] + rotated_translation[index]
                for index in range(3)
            )
            if delta.orientation_xyzw != (0.0, 0.0, 0.0, 1.0):
                raise ValueError(
                    f"{target_ref} 非 reference_target 坐标的旋转增量不受支持；"
                    "请发布绝对 orientation_xyzw"
                )
            goal = RigidTransform(translated, base_pose.orientation_xyzw)
        return ResolvedCartesianTarget(
            target_ref,
            CartesianPose(
                self.model.base_frame,
                goal.translation_m,
                goal.orientation_xyzw,
            ),
            (*source_chain, target_ref),
            ik_seed,
        )

    def _frame_transform(self, target_ref: str, frame_ref: str) -> RigidTransform:
        """返回来源坐标到机械臂基座的已校准静态变换。"""

        if not frame_ref.strip():
            raise ValueError(f"{target_ref} frame_ref 不能为空")
        try:
            return self._frame_transforms[frame_ref]
        except KeyError as exc:
            raise ValueError(
                f"{target_ref} frame_ref={frame_ref!r} 缺少精确安装标定"
            ) from exc


def transform_value(value: Any, target_ref: str) -> RigidTransform:
    """解析目标中的米制平移和 XYZW 四元数，并拒绝非单位四元数。"""

    data = _mapping(value, f"{target_ref}.value")
    xyz = numeric_sequence(data.get("xyz_m"), f"{target_ref}.value.xyz_m")
    orientation = numeric_sequence(
        data.get("orientation_xyzw"),
        f"{target_ref}.value.orientation_xyzw",
    )
    if len(xyz) != 3 or len(orientation) != 4:
        raise ValueError(f"{target_ref} 必须包含 xyz_m[3] 和 orientation_xyzw[4]")
    norm = math.sqrt(sum(item * item for item in orientation))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{target_ref} orientation_xyzw 必须是单位四元数")
    return RigidTransform(xyz, orientation)


def numeric_sequence(value: Any, field: str) -> tuple[float, ...]:
    """把 YAML 数值列表转换为有限浮点元组。"""

    if isinstance(value, (str, bytes)):
        raise TypeError(f"{field} 必须是数值列表")
    try:
        normalized = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是数值列表") from exc
    if not all(math.isfinite(item) for item in normalized):
        raise ValueError(f"{field} 只能包含有限数")
    return normalized


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    """要求一个配置字段是对象。"""

    if not isinstance(value, Mapping):
        raise TypeError(f"{field} 必须是对象")
    return value


__all__ = [
    "ArmTargetModel",
    "ArmTargetResolver",
    "CartesianPose",
    "JointSpecification",
    "JointType",
    "ResolvedCartesianTarget",
    "ResolvedJointTarget",
    "ResolvedMotionTarget",
    "ToolContext",
    "numeric_sequence",
    "transform_value",
]
