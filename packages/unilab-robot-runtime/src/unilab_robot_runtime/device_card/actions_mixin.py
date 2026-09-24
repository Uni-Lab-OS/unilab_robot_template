"""机械臂设备卡片公开动作 mixin（template 通用）。"""

from __future__ import annotations

import math
import uuid
from typing import TypedDict

from unilabos.registry.annotations import JSONValue
from unilabos.registry.decorators import action

from unilab_robot_runtime.vision_calibration.paths import VisionCalibrationPaths

from .context import ArmCardContext
from . import manual_motion
from . import vision_actions
from .observation_types import ExecutorObservation
from .point_catalog import point_set_revision_from_path, read_point_catalog_from_observation
from .types import RobotDebugSnapshot, RobotManualMotionResult, RobotTeachPointResult

_POINTSET_HOME_REF = "global.arm.home"


class RobotQueryResult(TypedDict):
    pose: list[float]
    joint: list[float]
    robot_mode: int | None
    command_id: int | None
    last_action: int
    tool_state: dict[str, JSONValue]
    mounted_tool: int
    suction_on: bool


class RailMountedArmCardMixin:
    """把 MoveIt 维护绑定投影成卡片可调用的设备动作（Action）。"""

    def get_moveit_online(self) -> bool:
        binding = self._card_binding_optional()
        port = getattr(binding, "commissioning_port", None) if binding is not None else None
        if port is None:
            return False
        try:
            return bool(port.commissioning_snapshot().online)
        except Exception:
            return False

    @action(description="读取机器人状态", always_free=True)
    def query(self) -> RobotQueryResult:
        snapshot = self._card_port().commissioning_snapshot()
        joints = snapshot.joint_positions or ()
        pose = snapshot.tcp_pose
        xyz_mm = (
            [value * 1000.0 for value in pose.xyz_m]
            if pose is not None
            else [0.0, 0.0, 0.0]
        )
        return {
            "pose": [*xyz_mm, 0.0, 0.0, 0.0],
            "joint": [math.degrees(item.position_si) for item in joints],
            "robot_mode": 5 if snapshot.online else 0,
            "command_id": None,
            "last_action": 0,
            "tool_state": {},
            "mounted_tool": 0,
            "suction_on": False,
        }

    @action(description="回原点")
    def home(self) -> None:
        self._execute_point_set_target(_POINTSET_HOME_REF, command_kind="home")

    @action(description="移动到活动 PointSet 锚点")
    def move_to_anchor(self, point_id: str) -> None:
        self._execute_point_set_target(point_id, command_kind="anchor")

    @action(description="读取 PointSet 目录与执行器观测", always_free=True)
    def read_point_catalog(self) -> RobotDebugSnapshot:
        context = self._arm_card_context()
        binding = self._card_binding_optional()
        if binding is not None:
            try:
                snapshot = manual_motion.read_debug_snapshot(binding, context)
                if not snapshot.get("joint_positions"):
                    fallback = self._read_point_catalog_without_moveit(context, binding)
                    merged = dict(snapshot)
                    merged["joint_positions"] = fallback.get("joint_positions", [])
                    return merged  # type: ignore[return-value]
                return snapshot
            except Exception:
                pass
        return self._read_point_catalog_without_moveit(context, binding)

    @action(description="读取机械臂调试卡片快照", always_free=True)
    def read_debug_snapshot(self) -> RobotDebugSnapshot:
        return manual_motion.read_debug_snapshot(
            self._card_binding(),
            self._arm_card_context(),
        )

    @action(description="离散关节点动")
    def jog_joint_once(
        self,
        joint_ref: str,
        direction: str,
        step_deg: float = 1.0,
    ) -> RobotManualMotionResult:
        return manual_motion.jog_joint_once(
            self._card_binding(),
            self._arm_card_context(),
            device_id=self._card_device_id(),
            monotonic_sequence=self._next_card_sequence(),
            joint_ref=joint_ref,
            direction=direction,
            step_deg=step_deg,
        )

    @action(description="离散工具中心点点动")
    def jog_tcp_once(
        self,
        axis: str,
        direction: str,
        step: float = 1.0,
        frame_ref: str = "arm_base",
    ) -> RobotManualMotionResult:
        return manual_motion.jog_tcp_once(
            self._card_binding(),
            self._arm_card_context(),
            device_id=self._card_device_id(),
            monotonic_sequence=self._next_card_sequence(),
            axis=axis,
            direction=direction,
            step=step,
            frame_ref=frame_ref,
        )

    @action(description="把当前关节位置写回作者 PointSet")
    def teach_point_from_current(
        self,
        target_ref: str,
        confirm: bool = False,
    ) -> RobotTeachPointResult:
        return manual_motion.teach_point_from_current(
            self._card_binding(),
            self._arm_card_context(),
            target_ref=target_ref,
            confirm=confirm,
        )

    @action(description="移动导轨到绝对位置", always_free=True)
    def move_rail_to_position(self, position_mm: float) -> None:
        manual_motion.move_rail_to_position(
            self._card_binding(),
            self._arm_card_context(),
            position_mm=position_mm,
        )

    @action(description="记录当前机械臂与导轨位置到 PointSet v3")
    def record_current_point(
        self,
        target_ref: str,
        expected_revision: str | None = None,
        confirm: bool = False,
        include_vision: bool = False,
        marker_ref: str | None = None,
    ) -> RobotTeachPointResult:
        return manual_motion.record_current_point(
            self._card_binding(),
            self._arm_card_context(),
            target_ref=target_ref,
            expected_revision=expected_revision,
            confirm=confirm,
            include_vision=include_vision,
            marker_ref=marker_ref,
        )

    @action(description="登记摄像头外参标定占位结果")
    def calibrate_camera_extrinsic(self, confirm: bool = False) -> dict[str, JSONValue]:
        paths = self._vision_calibration_paths()
        if paths is None:
            raise RuntimeError("当前设备未配置 vision 标定资产路径")
        return vision_actions.calibrate_camera_extrinsic(
            self._card_binding(),
            paths,
            confirm=confirm,
        )

    @action(description="登记 TCP 标定占位结果")
    def calibrate_tcp(self, confirm: bool = False) -> dict[str, JSONValue]:
        paths = self._vision_calibration_paths()
        if paths is None:
            raise RuntimeError("当前设备未配置 vision 标定资产路径")
        return vision_actions.calibrate_tcp(
            self._card_binding(),
            paths,
            confirm=confirm,
        )

    @action(description="为指定 process_warehouse 登记 marker")
    def record_marker(
        self,
        warehouse_ref: str,
        confirm: bool = False,
    ) -> dict[str, JSONValue]:
        paths = self._vision_calibration_paths()
        if paths is None:
            raise RuntimeError("当前设备未配置 vision 标定资产路径")
        return vision_actions.record_marker(
            self._card_binding(),
            paths,
            warehouse_ref=warehouse_ref,
            confirm=confirm,
        )

    def _arm_card_context(self) -> ArmCardContext:
        raise NotImplementedError("子类必须实现 _arm_card_context()")

    def _card_binding(self) -> object:
        self._ensure_card_binding()
        binding = self._card_binding_optional()
        if binding is None:
            raise RuntimeError("MoveIt 调试端口未就绪")
        return binding

    def _card_binding_optional(self) -> object | None:
        return getattr(self, "_moveit_split_binding", None) or getattr(
            self, "_moveit_binding", None
        )

    def _ensure_card_binding(self) -> None:
        """子类可覆盖以懒绑定 MoveIt 调试端口。"""

    def _card_port(self) -> object:
        port = getattr(self._card_binding(), "commissioning_port", None)
        if port is None:
            raise RuntimeError("MoveIt 调试绑定缺少 commissioning_port")
        return port

    def _card_device_id(self) -> str:
        return str(getattr(self, "device_id", "robot"))

    def _next_card_sequence(self) -> int:
        current = int(getattr(self, "_arm_card_sequence", 0)) + 1
        setattr(self, "_arm_card_sequence", current)
        return current

    def _vision_calibration_paths(self) -> VisionCalibrationPaths | None:
        return None

    def _execute_point_set_target(self, point_id: str, *, command_kind: str) -> None:
        from unilab_robot_contracts import CommandState, MoveTargetCommand

        normalized = str(point_id).strip()
        if not normalized:
            raise ValueError("PointSet 锚点不能为空")
        context = self._arm_card_context()
        binding = self._card_binding()
        port = self._card_port()
        resolve_anchor = getattr(port, "resolve_anchor", None)
        if callable(resolve_anchor):
            target = resolve_anchor(normalized)
        else:
            resolver = self._point_set_resolver()
            if resolver is None:
                raise RuntimeError("当前调试端口缺少 PointSet 解析器")
            target = resolver.resolve_arm_target(normalized)
        scale = min(
            float(port.commissioning_velocity_limit),
            float(port.commissioning_acceleration_limit),
        )
        device_id = self._card_device_id()
        prefix = context.boot_id_prefix
        command = MoveTargetCommand(
            command_id=f"card-{command_kind}-{uuid.uuid4()}",
            hardware_profile_digest=str(port.hardware_profile_digest),
            source_boot_id=f"{prefix}:{device_id}:card-{command_kind}",
            monotonic_sequence=self._next_card_sequence(),
            motion_profile_ref=context.default_motion_profile_ref(port),
            velocity_scale=scale,
            acceleration_scale=scale,
            target_ref=target.target_ref,
            target_revision=str(port.commissioning_target_revision),
        )
        session = binding.open_maintenance_session(f"{prefix}:{device_id}:{command_kind}")
        result = session.execute(command)
        if not result.state.terminal:
            raise RuntimeError(result.message)
        session.close()
        if result.state is not CommandState.SUCCEEDED:
            raise RuntimeError(result.message)

    def _point_set_resolver(self) -> object | None:
        return None

    def _read_point_catalog_without_moveit(
        self,
        context: ArmCardContext,
        binding: object | None = None,
    ) -> RobotDebugSnapshot:
        return read_point_catalog_from_observation(  # type: ignore[return-value]
            context,
            self._offline_catalog_observation(context, binding),
            point_set_revision=self._resolve_point_set_revision(context),
            catalog_port=self,
            moveit_port=self._moveit_catalog_port_optional(),
        )

    def _observation_joint_positions_si(
        self,
        binding: object | None,
    ) -> tuple[list[str], list[float]]:
        if binding is None:
            return [], []
        port = getattr(binding, "commissioning_port", None)
        if port is None:
            return [], []
        try:
            snapshot = port.commissioning_snapshot()
        except Exception:
            return [], []
        joints = getattr(snapshot, "joint_positions", None) or ()
        refs: list[str] = []
        positions: list[float] = []
        for item in joints:
            joint_ref = str(getattr(item, "joint_ref", "")).strip()
            position_si = getattr(item, "position_si", None)
            if not joint_ref or position_si is None:
                continue
            refs.append(joint_ref)
            positions.append(float(position_si))
        return refs, positions

    def _resolve_point_set_revision(self, context: ArmCardContext) -> str:
        resolver = self._point_set_resolver()
        if resolver is not None:
            revision = str(getattr(resolver, "revision", "")).strip()
            if revision:
                return revision
        if context.point_set_path is not None:
            return point_set_revision_from_path(context.point_set_path)
        return ""

    def _offline_catalog_observation(
        self,
        context: ArmCardContext,
        binding: object | None = None,
    ) -> ExecutorObservation:
        joint_refs, joint_positions_si = self._observation_joint_positions_si(binding)
        if not joint_refs:
            joint_refs = context.resolve_joint_catalog_refs()
            joint_positions_si = [0.0] * len(joint_refs)
        return {
            "source": "point_set",
            "online": binding is not None,
            "idle": True,
            "stale": True,
            "observed_at": 0.0,
            "execution_fenced": True,
            "joint_refs": joint_refs,
            "joint_positions_si": joint_positions_si,
        }

    def _moveit_catalog_port_optional(self) -> object | None:
        binding = self._card_binding_optional()
        if binding is None:
            return None
        return getattr(binding, "commissioning_port", None)
