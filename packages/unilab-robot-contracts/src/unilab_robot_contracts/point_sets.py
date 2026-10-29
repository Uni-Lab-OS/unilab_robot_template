"""WorkCell 级 PointSet v3 的唯一加载、展开和摘要合同。"""

from __future__ import annotations

import copy
import math
import re
from collections.abc import Mapping, Sequence
from io import StringIO
from typing import Any

import yaml

from .geometry import RigidTransform
from .grid import RailTargetModel, resolve_affine_grid
from .point_set_types import (
    AccessMotionBlock,
    InstallationCalibration,
    PointSetCatalogEntry,
    ResolvedPointTarget,
    ResolvedRailTarget,
)
from .point_set_validation import (
    DEVICE_REF_PATTERN,
    RESERVED_POINT_SET_KEYS,
    arm_digest_payload,
    mapping,
    stable_digest,
    string_tuple,
    target_mapping,
    validate_calibration_ref,
    validate_components,
    vector3,
)
from .point_set_validation import source_digest as calculate_source_digest
from .targets import (
    ArmTargetModel,
    ArmTargetResolver,
    ResolvedCartesianTarget,
    ResolvedMotionTarget,
    ToolContext,
    numeric_sequence,
)

_REVISION_PATTERN = re.compile(r"^(?P<name>.+)@(?P<version>\d+(?:\.\d+)*)$")
_INTERACTION_SUFFIXES = (".interaction_seed", ".interaction")


class RobotPointSetResolver:
    """独占 PointSet v3、设备分组、阵列和接近运动块语义的深模块。"""

    def __init__(
        self,
        point_set: Mapping[str, Any],
        *,
        arm_model: ArmTargetModel,
        tool_context: ToolContext,
        calibration: InstallationCalibration,
        rail_model: RailTargetModel | None = None,
        source_digest: str | None = None,
    ) -> None:
        """解析唯一的 `unilab.robot-point-set/v3` 作者资产。

        参数：PointSet、精确机械臂/工具/安装标定和可选导轨型号；
        ``source_digest`` 应使用发布原始字节摘要。返回：无。异常：Schema、
        身份、设备局部坐标、阵列或组件不兼容时关闭失败。加载不会触发运动、
        改变库位占用（SiteOccupancy）或解除执行 Fence。
        """

        if point_set.get("schema") != "unilab.robot-point-set/v3":
            raise ValueError("point-set schema 必须为 unilab.robot-point-set/v3")
        if "joint_names" in point_set or "unit" in point_set:
            raise ValueError("PointSet v3 禁止重复 joint_names 或顶层 unit")
        self.revision = str(point_set.get("revision", "")).strip()
        if not self.revision:
            raise ValueError("PointSet v3 revision 不能为空")
        self.source_digest = calculate_source_digest(point_set, source_digest)
        self.arm_model = arm_model
        self.tool_context = tool_context
        self.calibration = calibration
        self.rail_model = rail_model
        validate_components(
            point_set.get("components"),
            arm_model=arm_model,
            tool_context=tool_context,
            rail_model=rail_model,
        )
        validate_calibration_ref(
            point_set.get("installation_calibration"),
            calibration,
        )
        self._arm_raw: dict[str, Mapping[str, Any]] = {}
        self._source_points: dict[str, str] = {}
        self._rail_targets: dict[str, ResolvedRailTarget] = {}
        self._point_targets: dict[str, ResolvedPointTarget] = {}
        self._access_raw: dict[str, tuple[Mapping[str, Any], str]] = {}
        self._collect_global(point_set.get("global"))
        self._collect_device_groups(point_set)
        frame_transforms = {
            **calibration.frame_transforms,
            arm_model.base_frame: RigidTransform.identity(),
            "arm_base": RigidTransform.identity(),
        }
        self._arm_resolver = ArmTargetResolver(
            self._arm_raw,
            model=arm_model,
            tool_context=tool_context,
            frame_transforms=frame_transforms,
        )
        self._arm_targets = self._arm_resolver.resolve_all()
        self._finish_point_targets()

    @property
    def arm_targets(self) -> Mapping[str, ResolvedMotionTarget]:
        """返回所有已展开且可直接交给机械臂 Adapter 的目标。"""

        return dict(self._arm_targets)

    @property
    def rail_targets(self) -> Mapping[str, ResolvedRailTarget]:
        """返回全部直接 SI 导轨目标；单机械臂 PointSet 返回空映射。"""

        return dict(self._rail_targets)

    def resolve(self, target_ref: str) -> ResolvedPointTarget:
        """按稳定 `<device-group>.<target>` 引用返回复合点位包。"""

        try:
            return self._point_targets[str(target_ref)]
        except KeyError as exc:
            raise ValueError(f"PointSet v3 缺少 target_ref: {target_ref}") from exc

    def resolve_all(self) -> Mapping[str, ResolvedPointTarget]:
        """返回全部复合点位包，并保留每个目标的解析摘要。"""

        return dict(self._point_targets)

    def resolve_arm_target(self, target_ref: str) -> ResolvedMotionTarget:
        """按稳定 target_ref 或作者层 ``source_point`` 返回已解析机械臂目标。"""

        key = str(target_ref).strip()
        if not key:
            raise ValueError("PointSet v3 target_ref 不能为空")
        resolved = self._arm_targets.get(key)
        if resolved is not None:
            return resolved
        sourced = self._source_points.get(key)
        if sourced is not None:
            return self._arm_targets[sourced]
        raise ValueError(f"PointSet v3 缺少 arm target_ref: {target_ref}")

    def authoring_catalog(self) -> tuple[PointSetCatalogEntry, ...]:
        """返回作者层 ``joint_positions`` 目录，不展开接近偏移或阵列派生点。"""

        expected = len(self.arm_model.joint_specs)
        entries: list[PointSetCatalogEntry] = []
        for target_ref, raw in self._arm_raw.items():
            kind = str(raw.get("type", "")).strip()
            if kind != "joint_positions":
                continue
            joints = numeric_sequence(raw.get("value"), f"{target_ref}.value")
            if len(joints) != expected:
                raise ValueError(
                    f"{target_ref} 关节数量与 exact Arm 型号不一致"
                )
            entries.append(
                PointSetCatalogEntry(
                    target_ref,
                    str(raw.get("source_point", "")).strip(),
                    kind,
                    True,
                    joints,
                    self._rail_for_arm_ref(target_ref),
                    _catalog_group_ref(target_ref),
                )
            )
        return tuple(sorted(entries, key=lambda item: item.target_ref))

    def _rail_for_arm_ref(self, arm_ref: str) -> float | None:
        """把作者层机械臂引用关联到同一复合点的导轨 SI 位置。"""

        if arm_ref in self._rail_targets:
            return self._rail_targets[arm_ref].position_si
        compound = _compound_point_ref(arm_ref)
        point = self._point_targets.get(compound)
        if point is not None:
            return point.rail_position_si
        return None

    def _collect_global(self, value: Any) -> None:
        """收集不属于 Device 或库位（Site）的类型化全局目标。"""

        global_targets = mapping(value, "global")
        if "device_ref" in global_targets:
            raise ValueError("global 是保留作用域，禁止声明 device_ref")
        unknown = set(global_targets).difference({"arm", "rail"})
        if unknown:
            raise ValueError(f"global 含未知类型: {sorted(unknown)}")
        arm_targets = mapping(global_targets.get("arm", {}), "global.arm")
        for name, raw in arm_targets.items():
            target_ref = f"global.arm.{name}"
            self._register_arm_raw(target_ref, target_mapping(raw, target_ref))
        rail_targets = mapping(global_targets.get("rail", {}), "global.rail")
        for name, raw in rail_targets.items():
            target_ref = f"global.rail.{name}"
            self._rail_targets[target_ref] = ResolvedRailTarget(
                target_ref,
                self._rail_position(raw, target_ref),
                (target_ref,),
            )

    def _collect_device_groups(self, point_set: Mapping[str, Any]) -> None:
        """收集顶层可读设备分组，并拒绝缺失或重复稳定 device_ref。"""

        seen_device_refs: set[str] = set()
        for group_name, group_value in point_set.items():
            if group_name in RESERVED_POINT_SET_KEYS:
                continue
            group_ref = str(group_name).strip()
            if not group_ref:
                raise ValueError("PointSet v3 设备分组键不能为空")
            group = mapping(group_value, group_ref)
            device_ref = str(group.get("device_ref", "")).strip()
            if DEVICE_REF_PATTERN.fullmatch(device_ref) is None:
                raise ValueError(f"{group_ref}.device_ref 不是稳定 package-local 引用")
            if device_ref in seen_device_refs:
                raise ValueError(f"重复 device_ref: {device_ref}")
            seen_device_refs.add(device_ref)
            self._collect_transit_targets(group_ref, group.get("transit", {}))
            has_targets = "targets" in group
            has_grid = "grid" in group
            if has_targets == has_grid:
                raise ValueError(f"{group_ref} 必须且只能声明 targets 或 grid")
            if has_targets:
                self._collect_explicit_targets(group_ref, device_ref, group)
            else:
                self._collect_grid(group_ref, device_ref, group)
        if not seen_device_refs:
            raise ValueError("PointSet v3 至少包含一个设备分组")

    def _collect_transit_targets(self, group_ref: str, value: Any) -> None:
        """收集设备分组自有、但不直接代表库位交互的绝对避障点。"""

        transit = mapping(value, f"{group_ref}.transit")
        for name, raw in transit.items():
            target_name = str(name).strip()
            if not target_name:
                raise ValueError(f"{group_ref}.transit 名称不能为空")
            target_ref = f"{group_ref}.transit.{target_name}"
            if target_ref in self._arm_raw:
                raise ValueError(f"重复 transit target_ref: {target_ref}")
            self._register_arm_raw(target_ref, target_mapping(raw, target_ref))

    def _collect_explicit_targets(
        self,
        group_ref: str,
        device_ref: str,
        group: Mapping[str, Any],
    ) -> None:
        """收集普通设备下的显式复合点位包。"""

        targets = mapping(group.get("targets"), f"{group_ref}.targets")
        group_access = group.get("access")
        for name, target_value in targets.items():
            target_name = str(name).strip()
            target_ref = f"{group_ref}.{target_name}"
            target = mapping(target_value, target_ref)
            if not target_name or target_ref in self._point_targets:
                raise ValueError(f"重复或空 PointSet target_ref: {target_ref}")
            arm_raw = target.get("arm")
            access_raw = target.get("access", group_access)
            arm_target_ref: str | None = None
            if arm_raw is not None:
                arm_target_ref = (
                    f"{target_ref}.interaction" if access_raw is not None else target_ref
                )
                normalized_arm = target_mapping(
                    arm_raw,
                    f"{target_ref}.arm",
                )
                if (
                    access_raw is not None
                    and str(normalized_arm.get("type", "")) == "joint_positions"
                ):
                    seed_ref = f"{target_ref}.interaction_seed"
                    access = mapping(access_raw, f"{target_ref}.access")
                    frame_ref = str(access.get("frame_ref", "")).strip()
                    if not frame_ref:
                        raise ValueError(f"{target_ref}.access.frame_ref 不能为空")
                    self._register_arm_raw(seed_ref, normalized_arm)
                    self._register_arm_raw(
                        arm_target_ref,
                        {
                            "type": "cartesian_delta",
                            "relative_to": seed_ref,
                            "frame_ref": frame_ref,
                            "value": {
                                "xyz_m": [0.0, 0.0, 0.0],
                                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                            },
                        },
                    )
                else:
                    self._register_arm_raw(arm_target_ref, normalized_arm)
            rail_position = (
                None
                if "rail" not in target
                else self._rail_position(target["rail"], f"{target_ref}.rail")
            )
            if arm_target_ref is None and rail_position is None:
                raise ValueError(f"{target_ref} 必须至少包含 arm 或 rail")
            if rail_position is not None:
                self._rail_targets[target_ref] = ResolvedRailTarget(
                    target_ref,
                    rail_position,
                    (target_ref,),
                )
            if access_raw is not None:
                if arm_target_ref is None:
                    raise ValueError(f"{target_ref}.access 必须同时声明 arm interaction")
                self._collect_access(target_ref, access_raw, arm_target_ref)
            self._point_targets[target_ref] = ResolvedPointTarget(
                target_ref,
                device_ref,
                arm_target_ref,
                rail_position,
                None,
                (target_ref,),
                "",
            )

    def _collect_grid(
        self,
        group_ref: str,
        device_ref: str,
        group: Mapping[str, Any],
    ) -> None:
        """确定性展开阵列到内存目标索引，不生成逐格资产文件。"""

        grid_raw = mapping(group.get("grid"), f"{group_ref}.grid")
        cells = resolve_affine_grid(
            group_ref,
            grid_raw,
            rail_model=self.rail_model,
        )
        frame_ref = str(grid_raw["frame_ref"])
        access_raw = group.get("access")
        for cell_key, cell in cells.items():
            target_ref = f"{group_ref}.{cell_key}"
            arm_target_ref = (
                f"{target_ref}.interaction" if access_raw is not None else target_ref
            )
            self._register_arm_raw(
                arm_target_ref,
                {
                    "type": "cartesian_pose",
                    "frame_ref": frame_ref,
                    "value": {
                        "xyz_m": list(cell.xyz_m),
                        "orientation_xyzw": list(cell.orientation_xyzw),
                    },
                },
            )
            if cell.rail_position_si is not None:
                self._rail_targets[target_ref] = ResolvedRailTarget(
                    target_ref,
                    cell.rail_position_si,
                    cell.source_chain,
                )
            if access_raw is not None:
                self._collect_access(target_ref, access_raw, arm_target_ref)
            self._point_targets[target_ref] = ResolvedPointTarget(
                target_ref,
                device_ref,
                arm_target_ref,
                cell.rail_position_si,
                None,
                cell.source_chain,
                "",
            )

    def _collect_access(
        self,
        target_ref: str,
        value: Any,
        interaction_ref: str,
    ) -> None:
        """把 D9-1S 几何压缩展开为直接引用同一 interaction 的两个目标。"""

        raw = mapping(value, f"{target_ref}.access")
        if str(raw.get("type", "")) != "access_motion_block/v1":
            raise ValueError(f"{target_ref}.access.type 必须为 access_motion_block/v1")
        frame_ref = str(raw.get("frame_ref", "")).strip()
        if not frame_ref:
            raise ValueError(f"{target_ref}.access.frame_ref 不能为空")
        if str(raw.get("orientation_policy", "inherit_interaction")) != "inherit_interaction":
            raise ValueError("AccessMotionBlock v1 只允许 inherit_interaction 姿态策略")
        entry_offset = vector3(
            raw.get("entry_offset_xyz_m"),
            f"{target_ref}.access.entry_offset_xyz_m",
        )
        approach_offset = vector3(
            raw.get("approach_offset_xyz_m"),
            f"{target_ref}.access.approach_offset_xyz_m",
        )
        for phase, offset in (
            ("entry", entry_offset),
            ("approach", approach_offset),
        ):
            phase_ref = f"{target_ref}.{phase}"
            self._register_arm_raw(
                phase_ref,
                {
                    "type": "cartesian_delta",
                    "relative_to": interaction_ref,
                    "frame_ref": frame_ref,
                    "value": {
                        "xyz_m": list(offset),
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
            )
        self._access_raw[target_ref] = (raw, interaction_ref)

    def _finish_point_targets(self) -> None:
        """校验 transit 引用、构造接近块并冻结每个复合目标摘要。"""

        for target_ref, current in tuple(self._point_targets.items()):
            access_block = None
            if target_ref in self._access_raw:
                raw, interaction_ref = self._access_raw[target_ref]
                interaction = self._arm_targets[interaction_ref]
                if not isinstance(interaction, ResolvedCartesianTarget):
                    raise ValueError(
                        f"{target_ref}.access interaction 必须是绝对 TCP 位姿"
                    )
                transit_in = string_tuple(
                    raw.get("transit_in", ()),
                    f"{target_ref}.access.transit_in",
                )
                for transit_ref in transit_in:
                    if transit_ref not in self._arm_targets:
                        raise ValueError(
                            f"{target_ref}.access 引用了缺失 transit target: {transit_ref}"
                        )
                transit_out_raw = mapping(
                    raw.get("transit_out", {"mode": "reverse_transit_in"}),
                    f"{target_ref}.access.transit_out",
                )
                return_mode = str(transit_out_raw.get("mode", ""))
                if return_mode == "reverse_transit_in":
                    transit_out = tuple(reversed(transit_in))
                elif return_mode == "explicit":
                    transit_out = string_tuple(
                        transit_out_raw.get("targets"),
                        f"{target_ref}.access.transit_out.targets",
                    )
                    for transit_ref in transit_out:
                        if transit_ref not in self._arm_targets:
                            raise ValueError(
                                f"{target_ref}.access 引用了缺失 outbound target: "
                                f"{transit_ref}"
                            )
                else:
                    raise ValueError(
                        f"{target_ref}.access.transit_out.mode 只允许 "
                        "reverse_transit_in 或 explicit"
                    )
                access_block = AccessMotionBlock(
                    f"{target_ref}.access",
                    interaction_ref,
                    f"{target_ref}.approach",
                    f"{target_ref}.entry",
                    transit_in,
                    transit_out,
                    return_mode,
                )
            payload = {
                "point_set_revision": self.revision,
                "point_set_digest": self.source_digest,
                "calibration_revision": self.calibration.revision,
                "calibration_digest": self.calibration.digest,
                "target_ref": target_ref,
                "device_ref": current.device_ref,
                "arm": arm_digest_payload(
                    current.arm_target_ref,
                    self._arm_targets,
                    access_block,
                ),
                "rail_position_si": current.rail_position_si,
                "source_chain": current.source_chain,
            }
            self._point_targets[target_ref] = ResolvedPointTarget(
                target_ref,
                current.device_ref,
                current.arm_target_ref,
                current.rail_position_si,
                access_block,
                current.source_chain,
                stable_digest(payload),
            )

    def _register_arm_raw(self, target_ref: str, raw: Mapping[str, Any]) -> None:
        """登记一个机械臂作者目标，并索引可选的 ``source_point``。"""

        self._arm_raw[target_ref] = raw
        source = str(raw.get("source_point", "")).strip()
        if source:
            self._source_points.setdefault(source, target_ref)

    def _rail_position(self, value: Any, field: str) -> float:
        """解析普通目标的直接 SI 导轨位置并验证型号行程。"""

        if self.rail_model is None:
            raise ValueError(f"{field} 已声明，但 PointSet 未装配 rail component")
        raw = value
        if isinstance(value, Mapping):
            if set(value) != {"position_si"}:
                raise ValueError(f"{field} 只能包含 position_si")
            raw = value["position_si"]
        try:
            position = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field} 必须是直接 SI 位置") from exc
        lower, upper = self.rail_model.travel_m
        if not math.isfinite(position) or not lower <= position <= upper:
            raise ValueError(
                f"{field}={position} 超出 RailModel 行程 {self.rail_model.travel_m}"
            )
        return position


def revise_authored_joint_target(
    point_set: Mapping[str, Any],
    target_ref: str,
    joint_positions_si: Sequence[float],
    *,
    joint_count: int,
) -> dict[str, Any]:
    """把一条作者层 ``joint_positions`` 更新为新的型号顺序 SI 值并提升 revision。

    参数：完整 PointSet 映射、稳定 ``target_ref``、型号顺序关节 SI 和关节数量。
    返回：可写回 YAML 的新映射。异常：目标不是作者层关节、数量不匹配或
    revision 无法提升时失败关闭。安全：不展开或写入 AccessMotionBlock 派生点。
    """

    if not isinstance(joint_count, int) or isinstance(joint_count, bool) or joint_count < 1:
        raise ValueError("exact Arm 型号关节数量必须是正整数")
    joints = numeric_sequence(joint_positions_si, "joint_positions_si")
    if len(joints) != joint_count:
        raise ValueError("示教关节数量与 exact Arm 型号不一致")
    revised = _mutable_mapping(point_set)
    node = _authored_joint_node(revised, target_ref)
    existing = numeric_sequence(node.get("value"), f"{target_ref}.value")
    if len(existing) != joint_count:
        raise ValueError(f"{target_ref} 关节数量与 exact Arm 型号不一致")
    node["value"] = [float(item) for item in joints]
    revised["revision"] = bump_point_set_revision(str(revised.get("revision", "")))
    return revised


def patch_authored_joint_target_yaml(
    text: str,
    target_ref: str,
    joint_positions_si: Sequence[float],
    *,
    joint_count: int,
) -> str:
    """在原文上只替换 ``revision`` 和目标 ``value`` 标量，保留其余注释与排版。

    参数：PointSet YAML 原文、稳定 ``target_ref``、型号顺序关节 SI 和关节数量。
    返回：可原子写回的新文本。异常：目标不可写、节点不是标量序列或
    revision 无法提升时失败关闭。安全：不重排其它点位，不展开派生点。
    """

    loaded = yaml.safe_load(text)
    if not isinstance(loaded, Mapping):
        raise TypeError("PointSet YAML 根节点必须是对象")
    revised = revise_authored_joint_target(
        loaded,
        target_ref,
        joint_positions_si,
        joint_count=joint_count,
    )
    path, _node = _authored_joint_location(loaded, target_ref)
    root = yaml.compose(StringIO(text), Loader=yaml.SafeLoader)
    if not isinstance(root, yaml.MappingNode):
        raise TypeError("PointSet YAML 根节点必须是对象")
    revision_node = _compose_child(root, "revision")
    if not isinstance(revision_node, yaml.ScalarNode):
        raise ValueError("PointSet revision 必须是标量")
    value_node = root
    for key in (*path, "value"):
        value_node = _compose_child(value_node, key)
    if not isinstance(value_node, yaml.SequenceNode):
        raise ValueError(f"{target_ref}.value 必须是 YAML 序列")
    if len(value_node.value) != joint_count:
        raise ValueError(f"{target_ref}.value 关节数量与 exact Arm 型号不一致")
    updated = revised
    for key in path:
        updated = updated[key]
    new_values = numeric_sequence(updated.get("value"), f"{target_ref}.value")
    replacements = [
        (
            revision_node.start_mark.index,
            revision_node.end_mark.index,
            str(revised["revision"]),
        )
    ]
    for item_node, number in zip(value_node.value, new_values, strict=True):
        if not isinstance(item_node, yaml.ScalarNode):
            raise ValueError(f"{target_ref}.value 必须全部是标量")
        replacements.append(
            (
                item_node.start_mark.index,
                item_node.end_mark.index,
                _format_joint_si(number),
            )
        )
    replacements.sort(key=lambda item: item[0], reverse=True)
    patched = text
    for start, end, token in replacements:
        patched = patched[:start] + token + patched[end:]
    return patched


def patch_authored_composite_target_yaml(
    text: str,
    target_ref: str,
    joint_positions_si: Sequence[float],
    rail_position_si: float | None,
    *,
    joint_count: int,
    expected_revision: str | None = None,
) -> str:
    """原子更新一个 PointSet v3 作者目标的机械臂与可选导轨位置。

    参数：YAML 原文、稳定复合目标引用、型号顺序关节 SI、可选导轨 SI、
    exact Arm 关节数和调用方已读取的修订。返回：只提升一次修订的新原文。
    异常：修订冲突、目标不可写、导轨节点缺失或数值无效时失败关闭。
    """

    loaded = yaml.safe_load(text)
    if not isinstance(loaded, Mapping):
        raise TypeError("PointSet YAML 根节点必须是对象")
    current_revision = str(loaded.get("revision", "")).strip()
    if expected_revision is not None and str(expected_revision).strip() != current_revision:
        raise ValueError(
            f"PointSet 修订冲突: expected={expected_revision!r}, actual={current_revision!r}"
        )
    revised = revise_authored_joint_target(
        loaded,
        target_ref,
        joint_positions_si,
        joint_count=joint_count,
    )
    arm_path, _arm_node = _authored_joint_location(loaded, target_ref)
    rail_path = (*arm_path[:-1], "rail", "position_si")
    rail_value: float | None = None
    if rail_position_si is not None:
        rail_value = float(rail_position_si)
        if not math.isfinite(rail_value):
            raise ValueError("rail_position_si 必须是有限数")
        node: Any = revised
        for key in rail_path[:-1]:
            node = _mapping_dict(node.get(key), ".".join(rail_path[:-1]))
        if rail_path[-1] not in node:
            raise ValueError(f"{target_ref} 没有可写回的 rail.position_si")
        node[rail_path[-1]] = rail_value

    root = yaml.compose(StringIO(text), Loader=yaml.SafeLoader)
    if not isinstance(root, yaml.MappingNode):
        raise TypeError("PointSet YAML 根节点必须是对象")
    replacements: list[tuple[int, int, str]] = []
    revision_node = _compose_child(root, "revision")
    replacements.append(
        (revision_node.start_mark.index, revision_node.end_mark.index, str(revised["revision"]))
    )
    value_node = root
    for key in (*arm_path, "value"):
        value_node = _compose_child(value_node, key)
    if not isinstance(value_node, yaml.SequenceNode) or len(value_node.value) != joint_count:
        raise ValueError(f"{target_ref}.value 关节数量与 exact Arm 型号不一致")
    joints = numeric_sequence(joint_positions_si, "joint_positions_si")
    for item_node, number in zip(value_node.value, joints, strict=True):
        replacements.append(
            (item_node.start_mark.index, item_node.end_mark.index, _format_joint_si(number))
        )
    if rail_value is not None:
        rail_node = root
        for key in rail_path:
            rail_node = _compose_child(rail_node, key)
        if not isinstance(rail_node, yaml.ScalarNode):
            raise ValueError(f"{target_ref}.rail.position_si 必须是标量")
        replacements.append(
            (rail_node.start_mark.index, rail_node.end_mark.index, _format_joint_si(rail_value))
        )
    patched = text
    for start, end, token in sorted(replacements, reverse=True):
        patched = patched[:start] + token + patched[end:]
    return patched


def bump_point_set_revision(revision: str) -> str:
    """把 ``name@x.y.z`` 的最后一段数字加一，保持作者身份前缀。"""

    match = _REVISION_PATTERN.fullmatch(str(revision).strip())
    if match is None:
        raise ValueError("PointSet revision 必须为 name@数字版本")
    parts = match.group("version").split(".")
    parts[-1] = str(int(parts[-1]) + 1)
    return f"{match.group('name')}@{'.'.join(parts)}"


def _authored_joint_node(point_set: dict[str, Any], target_ref: str) -> dict[str, Any]:
    """定位可写回的作者层 ``joint_positions`` 节点，拒绝派生接近点。"""

    _path, node = _authored_joint_location(point_set, target_ref)
    return node


def _authored_joint_location(
    point_set: Mapping[str, Any],
    target_ref: str,
) -> tuple[tuple[str, ...], dict[str, Any]]:
    """返回作者层关节节点的文档键路径及其可写映射。"""

    key = str(target_ref).strip()
    if not key:
        raise ValueError("PointSet v3 target_ref 不能为空")
    global_arm = _mapping_dict(
        _mapping_dict(point_set.get("global"), "global").get("arm", {}),
        "global.arm",
    )
    prefix = "global.arm."
    if key.startswith(prefix):
        name = key[len(prefix):]
        return ("global", "arm", name), _require_joint_node(global_arm.get(name), key)
    for group_name, group_value in point_set.items():
        if group_name in RESERVED_POINT_SET_KEYS:
            continue
        group = _mapping_dict(group_value, group_name)
        transit = _mapping_dict(group.get("transit", {}), f"{group_name}.transit")
        transit_prefix = f"{group_name}.transit."
        if key.startswith(transit_prefix):
            name = key[len(transit_prefix):]
            return (
                (group_name, "transit", name),
                _require_joint_node(transit.get(name), key),
            )
        targets = _mapping_dict(group.get("targets", {}), f"{group_name}.targets")
        group_access = group.get("access")
        for name, target_value in targets.items():
            target = _mapping_dict(target_value, f"{group_name}.{name}")
            arm = target.get("arm")
            if not isinstance(arm, dict):
                continue
            compound = f"{group_name}.{name}"
            has_access = target.get("access", group_access) is not None
            authored = f"{compound}.interaction_seed" if has_access else compound
            if key in {authored, compound, f"{compound}.interaction"}:
                return (
                    (group_name, "targets", name, "arm"),
                    _require_joint_node(arm, key),
                )
    raise ValueError(f"PointSet v3 没有可写回的 joint_positions 目标: {key}")


def _compose_child(node: yaml.Node, key: str) -> yaml.Node:
    """按键取出 YAML compose 映射的子节点。"""

    if not isinstance(node, yaml.MappingNode):
        raise ValueError(f"PointSet YAML 路径 {key} 不是对象")
    for item_key, item_value in node.value:
        if isinstance(item_key, yaml.ScalarNode) and item_key.value == key:
            return item_value
    raise ValueError(f"PointSet YAML 缺少键 {key}")


def _format_joint_si(value: float) -> str:
    """把关节 SI 写成定点小数，去掉无意义的尾零。"""

    text = f"{float(value):.12f}".rstrip("0")
    if text.endswith("."):
        text += "0"
    if text == "-0.0":
        return "0.0"
    return text


def _require_joint_node(value: Any, target_ref: str) -> dict[str, Any]:
    """要求节点是可写的 ``joint_positions`` 对象。"""

    if not isinstance(value, dict) or str(value.get("type", "")) != "joint_positions":
        raise ValueError(f"PointSet v3 没有可写回的 joint_positions 目标: {target_ref}")
    return value


def _catalog_group_ref(target_ref: str) -> str:
    """把作者层机械臂引用投影为界面分组键。"""

    trimmed = _compound_point_ref(target_ref)
    if trimmed.startswith("global.arm."):
        return "global.arm"
    if ".transit." in trimmed:
        return trimmed.rsplit(".", 1)[0]
    return trimmed.rsplit(".", 1)[0]


def _compound_point_ref(target_ref: str) -> str:
    """去掉 interaction 后缀，得到复合点或全局/transit 引用。"""

    for suffix in _INTERACTION_SUFFIXES:
        if target_ref.endswith(suffix):
            return target_ref[: -len(suffix)]
    return target_ref


def _mutable_mapping(value: Mapping[str, Any]) -> dict[str, Any]:
    """深拷贝 PointSet 映射，使示教写回不修改调用方原对象。"""

    return copy.deepcopy(dict(value))


def _mapping_dict(value: Any, field: str) -> dict[str, Any]:
    """把 YAML 对象投影为可写 dict。"""

    raw = mapping(value, field)
    return raw if isinstance(raw, dict) else dict(raw)


__all__ = [
    "AccessMotionBlock",
    "InstallationCalibration",
    "PointSetCatalogEntry",
    "ResolvedPointTarget",
    "ResolvedRailTarget",
    "RobotPointSetResolver",
    "bump_point_set_revision",
    "patch_authored_composite_target_yaml",
    "patch_authored_joint_target_yaml",
    "revise_authored_joint_target",
]
