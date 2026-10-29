"""机械臂调试协议（Robot Commissioning Protocol）的纯合同测试。"""

from __future__ import annotations

import math
import time
import unittest
from dataclasses import replace

from unilab_robot_contracts import (
    COMMISSIONING_PROTOCOL_VERSION,
    AngleUnit,
    CartesianPose,
    CommandResult,
    CommandState,
    CommissioningCapabilities,
    CommissioningJointPosition,
    CommissioningMotionKind,
    CommissioningPoseInput,
    CommissioningSnapshot,
    ControlledStopCommand,
    EulerRotationOrder,
    JointJogCommand,
    MotionDirection,
    MovePoseCommand,
    MoveTargetCommand,
    ObservationState,
    RobotCommissioningPort,
    TcpAxis,
    TcpJogCommand,
)
from unilab_robot_contracts.geometry import RigidTransform


def _joint_jog(command_id: str = "joint-jog-1") -> JointJogCommand:
    """构造一个只引用稳定关节身份、不复制型号关节表的调试命令。"""

    return JointJogCommand(
        command_id=command_id,
        hardware_profile_digest="profile-digest",
        source_boot_id="test-boot",
        monotonic_sequence=1,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.1,
        acceleration_scale=0.1,
        joint_ref="joint_2",
        direction=MotionDirection.POSITIVE,
        step_si=math.radians(1.0),
    )


def _tcp_jog(command_id: str = "tcp-jog-1") -> TcpJogCommand:
    """构造一个在工具坐标系中沿 Z 轴负向移动 5 mm 的调试命令。"""

    return TcpJogCommand(
        command_id=command_id,
        hardware_profile_digest="profile-digest",
        source_boot_id="test-boot",
        monotonic_sequence=2,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.1,
        acceleration_scale=0.1,
        frame_ref="tool",
        axis=TcpAxis.Z,
        direction=MotionDirection.NEGATIVE,
        step_si=0.005,
    )


def _move_pose(command_id: str = "move-pose-1") -> MovePoseCommand:
    """构造一个使用毫米、角度和固定轴 XYZ 顺序的临时目标位姿命令。"""

    return MovePoseCommand(
        command_id=command_id,
        hardware_profile_digest="profile-digest",
        source_boot_id="test-boot",
        monotonic_sequence=3,
        motion_profile_ref="maintenance-slow",
        velocity_scale=0.1,
        acceleration_scale=0.1,
        pose_input=CommissioningPoseInput(
            frame_ref="arm_base",
            xyz_mm=(400.0, 100.0, 250.0),
            rotation_xyz=(10.0, 20.0, 30.0),
            angle_unit=AngleUnit.DEG,
            rotation_order=EulerRotationOrder.XYZ,
        ),
        tool_context_digest="tool-context-digest",
    )


class _ProtocolOnlyPort:
    """证明统一 Interface 不依赖 PLC、SDK 或 MoveIt 类型的结构化测试端口。"""

    @property
    def commissioning_capabilities(self) -> CommissioningCapabilities:
        """返回该测试端口声明的五类封闭调试能力。"""

        return CommissioningCapabilities(
            move_target=True,
            move_pose=True,
            tcp_jog=True,
            joint_jog=True,
            controlled_stop=True,
        )

    @property
    def commissioning_target_revision(self) -> str:
        """返回测试端口活动的固定目标版本。"""

        return "test-points@1.0.0"

    def commissioning_snapshot(self) -> CommissioningSnapshot:
        """返回包含完整关节状态和 TCP 位姿的新鲜只读快照。"""

        return CommissioningSnapshot(
            state=ObservationState.KNOWN,
            observed_at=time.time(),
            max_age_s=1.0,
            source="test:protocol-only",
            online=True,
            idle=True,
            active_command_id=None,
            execution_fenced=False,
            joint_positions=(
                CommissioningJointPosition("joint_1", 0.0),
                CommissioningJointPosition("joint_2", 0.0),
            ),
            tcp_pose=CartesianPose(
                "arm_base",
                (0.4, 0.1, 0.25),
                (0.0, 0.0, 0.0, 1.0),
            ),
        )

    def execute_commissioning(
        self,
        command: MoveTargetCommand
        | MovePoseCommand
        | TcpJogCommand
        | JointJogCommand
        | ControlledStopCommand,
    ) -> CommandResult:
        """返回同一命令身份的确定成功结果，不执行任何物理动作。"""

        return CommandResult(command.command_id, CommandState.SUCCEEDED, "test-only")


class CommissioningContractTests(unittest.TestCase):
    """验证统一调试 Interface 的身份、单位、限幅和状态语义。"""

    def test_one_port_covers_all_transport_independent_commands(self) -> None:
        """一个结构化 Interface 必须容纳五类命令且不出现传输类型字段。"""

        port = _ProtocolOnlyPort()

        self.assertIsInstance(port, RobotCommissioningPort)
        for kind in CommissioningMotionKind:
            self.assertTrue(port.commissioning_capabilities.supports(kind))
        self.assertTrue(port.commissioning_snapshot().is_fresh())

    def test_joint_jog_uses_model_owned_joint_ref_and_si_step(self) -> None:
        """单关节点动不得复制 joint_names、unit 或锁定容差配置。"""

        command = _joint_jog()
        payload = command.canonical_payload()

        self.assertEqual(command.kind, CommissioningMotionKind.JOINT_JOG)
        self.assertEqual(payload["schema_version"], COMMISSIONING_PROTOCOL_VERSION)
        self.assertEqual(payload["joint_ref"], "joint_2")
        self.assertNotIn("joint_names", payload)
        self.assertNotIn("unit", payload)
        self.assertNotIn("locked_joint_tolerance_si", payload)

    def test_tcp_jog_axis_determines_si_dimension_without_step_maximum(self) -> None:
        """TCP 平移和旋转共享 step_si，允许任意正的有限单步。"""

        translation = _tcp_jog()
        rotation = replace(
            translation,
            command_id="tcp-rz",
            axis=TcpAxis.RZ,
            step_si=math.radians(2.0),
        )

        self.assertFalse(translation.axis.rotational)
        self.assertTrue(rotation.axis.rotational)
        self.assertEqual(replace(translation, step_si=20.0).step_si, 20.0)
        self.assertEqual(
            replace(rotation, step_si=math.radians(720.0)).step_si,
            math.radians(720.0),
        )
        with self.assertRaisesRegex(ValueError, "正的有限数"):
            replace(translation, step_si=0.0)
        with self.assertRaisesRegex(ValueError, "正的有限数"):
            replace(rotation, step_si=math.inf)

    def test_all_tcp_jog_buttons_map_to_axis_and_direction_parameters(self) -> None:
        """十二个 TCP 点动按钮必须只是六个轴和两个方向的参数组合。"""

        base_command = _tcp_jog()
        # 此集合代表界面十二个按钮的稳定协议含义，而不是十二个公开方法。
        button_commands = {
            (axis.value, direction.value): replace(
                base_command,
                command_id=f"tcp-{axis.value}-{direction.value}",
                axis=axis,
                direction=direction,
                step_si=math.radians(1.0) if axis.rotational else 0.001,
            )
            for axis in TcpAxis
            for direction in MotionDirection
        }

        self.assertEqual(len(button_commands), 12)
        self.assertEqual(
            button_commands[("rx", "negative")].kind,
            CommissioningMotionKind.TCP_JOG,
        )

    def test_six_axis_joint_buttons_use_model_owned_joint_refs(self) -> None:
        """J1 至 J6 的正负按钮必须形成十二个单关节参数组合。"""

        base_command = _joint_jog()
        # ``joint_1`` 至 ``joint_6`` 模拟 CR7 exact Arm 型号提供的稳定关节引用。
        button_commands = {
            (joint_ref, direction.value): replace(
                base_command,
                command_id=f"{joint_ref}-{direction.value}",
                joint_ref=joint_ref,
                direction=direction,
            )
            for joint_ref in (f"joint_{index}" for index in range(1, 7))
            for direction in MotionDirection
        }

        self.assertEqual(len(button_commands), 12)
        self.assertNotIn(
            "joint_names",
            button_commands[("joint_6", "positive")].canonical_payload(),
        )

    def test_motion_scale_is_limited_even_before_adapter_exists(self) -> None:
        """协议层先把远程调试速度和加速度缩放限制在 25% 内。"""

        with self.assertRaisesRegex(ValueError, "velocity_scale"):
            replace(_joint_jog(), velocity_scale=0.5)
        with self.assertRaisesRegex(ValueError, "acceleration_scale"):
            replace(_joint_jog(), acceleration_scale=0.5)

    def test_move_target_binds_asset_revision(self) -> None:
        """点位移动必须同时绑定稳定目标引用和点位/程序资产版本。"""

        command = MoveTargetCommand(
            command_id="move-ready",
            hardware_profile_digest="profile-digest",
            source_boot_id="test-boot",
            monotonic_sequence=3,
            motion_profile_ref="maintenance-slow",
            velocity_scale=0.1,
            acceleration_scale=0.1,
            target_ref="ready",
            target_revision="cr7-moveit-sim@1.0.0",
        )

        self.assertEqual(command.kind, CommissioningMotionKind.MOVE_TARGET)
        self.assertEqual(command.canonical_payload()["target_revision"], "cr7-moveit-sim@1.0.0")

    def test_move_pose_normalizes_millimeters_and_degrees(self) -> None:
        """临时目标位姿必须把 mm 与 deg 唯一转换为 m 和 XYZW 四元数。"""

        command = _move_pose()
        resolved_pose = command.pose_input.resolved_pose
        expected_orientation = RigidTransform.from_rpy(
            (0.0, 0.0, 0.0),
            tuple(math.radians(value) for value in (10.0, 20.0, 30.0)),
        ).orientation_xyzw

        self.assertEqual(command.kind, CommissioningMotionKind.MOVE_POSE)
        self.assertEqual(resolved_pose.xyz_m, (0.4, 0.1, 0.25))
        for actual, expected in zip(
            resolved_pose.orientation_xyzw,
            expected_orientation,
            strict=True,
        ):
            self.assertAlmostEqual(actual, expected)
        payload = command.canonical_payload()
        self.assertEqual(payload["schema_version"], 2)
        self.assertEqual(payload["pose_input"]["angle_unit"], "deg")  # type: ignore[index]
        self.assertEqual(payload["pose_input"]["rotation_order"], "xyz")  # type: ignore[index]
        self.assertNotIn("target_ref", payload)

    def test_move_pose_supports_all_six_non_ambiguous_rotation_orders(self) -> None:
        """六种旋转顺序必须可选，并对非零三轴角产生各自的确定姿态。"""

        # 四元数集合是六种固定轴应用顺序的规范结果，用于阻止 Adapter 自行解释。
        orientations = {
            tuple(
                round(component, 12)
                for component in replace(
                    _move_pose().pose_input,
                    rotation_order=rotation_order,
                ).orientation_xyzw
            )
            for rotation_order in EulerRotationOrder
        }

        self.assertEqual(len(orientations), 6)
        for orientation in orientations:
            self.assertAlmostEqual(sum(value * value for value in orientation), 1.0)

    def test_snapshot_rejects_ambiguous_joint_identity(self) -> None:
        """调试快照不得用重复 joint_ref 返回无法确定的关节状态。"""

        with self.assertRaisesRegex(ValueError, "不得重复"):
            CommissioningSnapshot(
                state=ObservationState.KNOWN,
                observed_at=time.time(),
                max_age_s=1.0,
                source="test:duplicate-joint",
                online=True,
                idle=True,
                active_command_id=None,
                execution_fenced=False,
                joint_positions=(
                    CommissioningJointPosition("joint_1", 0.0),
                    CommissioningJointPosition("joint_1", 0.1),
                ),
            )


if __name__ == "__main__":
    unittest.main()
