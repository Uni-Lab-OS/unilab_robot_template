"""Preview 执行器 + PointSet/vision 维护（无 MoveIt binding）。"""

from __future__ import annotations

import json
from typing import TypedDict

from unilabos.registry.annotations import JSONValue
from unilabos.registry.decorators import action

from unilab_robot_runtime.vision_calibration.paths import VisionCalibrationPaths

from . import manual_motion
from . import vision_actions
from .context import ArmCardContext
from .observation_adapters import observation_from_preview_arm
from .point_catalog import point_set_revision_from_path, read_point_catalog_from_observation
from .types import RobotTeachPointResult


class PreviewSnapshotEnvelope(TypedDict):
    point_set_revision: str
    online: bool
    idle: bool
    snapshot_json: str


class PreviewArmCardMixin:
    """Preview 臂：示教走执行器，落盘走 PointSet/vision，不依赖 MoveIt。"""

    @action(description="读取 PointSet 目录与执行器观测", always_free=True)
    def read_point_catalog(self) -> dict[str, JSONValue]:
        catalog = self._read_point_catalog_snapshot()
        return catalog  # type: ignore[return-value]

    @action(description="Preview 卡片快照（过渡 alias）", always_free=True)
    def read_preview_snapshot(self) -> PreviewSnapshotEnvelope:
        catalog = self._read_point_catalog_snapshot()
        payload = {
            key: catalog[key]
            for key in catalog
            if key
            not in {
                "velocity_limit",
                "acceleration_limit",
                "active_command_id",
            }
        }
        joints = payload.pop("joint_positions", [])
        payload["joint_positions"] = [
            {
                "joint_ref": item.get("joint_ref", ""),
                "position_deg": item.get("position_deg", 0.0),
            }
            for item in joints
        ]
        sites = [
            {
                "site_ref": target.get("target_ref", target.get("source_point", "")),
                "label": target.get("source_point", ""),
                "group_ref": target.get("group_ref", ""),
                "xyz_m": [],
            }
            for target in payload.get("point_targets", [])
        ]
        caps = payload.get("capabilities", {})
        snapshot_payload = {
            "source": payload.get("source", "preview"),
            "joint_positions": payload.get("joint_positions", []),
            "sites": sites,
            "capabilities": {
                "joint_jog": caps.get("joint_jog", True),
                "tcp_jog": caps.get("tcp_jog", False),
                "rail_move": caps.get("rail_move", False),
                "composite_point_record": caps.get("composite_point_record", False),
            },
            "rail": payload.get("rail"),
            "calibration": payload.get("calibration"),
        }
        return PreviewSnapshotEnvelope(
            point_set_revision=str(payload.get("point_set_revision", "")),
            online=bool(payload.get("online")),
            idle=bool(payload.get("idle")),
            snapshot_json=json.dumps(snapshot_payload, ensure_ascii=False),
        )

    @action(description="Preview 离散 TCP 点动")
    def jog_tcp_once(
        self,
        axis: str,
        direction: str,
        step: float = 1.0,
        frame_ref: str = "arm_base",
    ) -> dict[str, JSONValue]:
        from unilab_robot_runtime.preview.preview_arm_device import PreviewArmDevice

        return PreviewArmDevice.jog_tcp_once(
            self,
            axis=axis,
            direction=direction,
            step=step,
            frame_ref=frame_ref,
        )  # type: ignore[return-value]

    @action(description="把当前关节位置写回作者 PointSet")
    def teach_point_from_current(
        self,
        target_ref: str,
        confirm: bool = False,
    ) -> RobotTeachPointResult:
        return manual_motion.teach_point_from_current(
            None,
            self._arm_card_context(),
            target_ref=target_ref,
            confirm=confirm,
            observation=self._executor_observation(),
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
            None,
            self._arm_card_context(),
            target_ref=target_ref,
            expected_revision=expected_revision,
            confirm=confirm,
            include_vision=include_vision,
            marker_ref=marker_ref,
            observation=self._executor_observation(),
        )

    @action(description="登记摄像头外参标定占位结果")
    def calibrate_camera_extrinsic(self, confirm: bool = False) -> dict[str, JSONValue]:
        paths = self._vision_calibration_paths()
        if paths is None:
            raise RuntimeError("当前设备未配置 vision 标定资产路径")
        return vision_actions.calibrate_camera_extrinsic_from_observation(
            self._executor_observation(),
            paths,
            confirm=confirm,
        )

    @action(description="登记 TCP 标定占位结果")
    def calibrate_tcp(self, confirm: bool = False) -> dict[str, JSONValue]:
        paths = self._vision_calibration_paths()
        if paths is None:
            raise RuntimeError("当前设备未配置 vision 标定资产路径")
        return vision_actions.calibrate_tcp_from_observation(
            self._executor_observation(),
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
        return vision_actions.record_marker_from_observation(
            self._executor_observation(),
            paths,
            warehouse_ref=warehouse_ref,
            confirm=confirm,
        )

    def _read_point_catalog_snapshot(self) -> dict[str, object]:
        context = self._arm_card_context()
        observation = self._executor_observation()
        revision = ""
        if context.point_set_path is not None:
            revision = point_set_revision_from_path(context.point_set_path)
        return read_point_catalog_from_observation(
            context,
            observation,
            point_set_revision=revision,
            catalog_port=self,
            calibration_meta=self._calibration_meta(),
        )

    def _executor_observation(self) -> object:
        raise NotImplementedError("子类必须实现 _executor_observation()")

    def _arm_card_context(self) -> ArmCardContext:
        raise NotImplementedError("子类必须实现 _arm_card_context()")

    def _vision_calibration_paths(self) -> VisionCalibrationPaths | None:
        return None

    def _calibration_meta(self) -> dict[str, object] | None:
        return None

    def _preview_device_id(self) -> str:
        return str(getattr(self, "device_id", "elite_left") or "elite_left")
