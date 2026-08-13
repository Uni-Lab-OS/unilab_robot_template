"""WorkCell 级 PointSet v3 的唯一加载、展开和摘要合同。"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from .geometry import RigidTransform
from .grid import RailTargetModel, resolve_affine_grid
from .point_set_types import (
    AccessMotionBlock,
    InstallationCalibration,
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
)


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
        """按内部或全局稳定引用返回已解析机械臂目标。"""

        try:
            return self._arm_targets[str(target_ref)]
        except KeyError as exc:
            raise ValueError(f"PointSet v3 缺少 arm target_ref: {target_ref}") from exc

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
            self._arm_raw[target_ref] = target_mapping(raw, target_ref)
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
            self._arm_raw[target_ref] = target_mapping(raw, target_ref)

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
                    self._arm_raw[seed_ref] = normalized_arm
                    self._arm_raw[arm_target_ref] = {
                        "type": "cartesian_delta",
                        "relative_to": seed_ref,
                        "frame_ref": frame_ref,
                        "value": {
                            "xyz_m": [0.0, 0.0, 0.0],
                            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        },
                    }
                else:
                    self._arm_raw[arm_target_ref] = normalized_arm
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
            self._arm_raw[arm_target_ref] = {
                "type": "cartesian_pose",
                "frame_ref": frame_ref,
                "value": {
                    "xyz_m": list(cell.xyz_m),
                    "orientation_xyzw": list(cell.orientation_xyzw),
                },
            }
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
            self._arm_raw[phase_ref] = {
                "type": "cartesian_delta",
                "relative_to": interaction_ref,
                "frame_ref": frame_ref,
                "value": {
                    "xyz_m": list(offset),
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            }
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


__all__ = [
    "AccessMotionBlock",
    "InstallationCalibration",
    "ResolvedPointTarget",
    "ResolvedRailTarget",
    "RobotPointSetResolver",
]
