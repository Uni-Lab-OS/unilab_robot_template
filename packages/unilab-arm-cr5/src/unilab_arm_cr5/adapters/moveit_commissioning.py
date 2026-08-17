"""CR5 的统一 Headless MoveIt 维护调试 Adapter。"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from typing import Any, Protocol

from unilab_robot_contracts import (
    CartesianPose,
    CommandResult,
    CommandState,
    CommissioningCapabilities,
    CommissioningCommand,
    CommissioningJointPosition,
    CommissioningSnapshot,
    ControlledStopCommand,
    JointJogCommand,
    MovePoseCommand,
    MoveTargetCommand,
    ObservationState,
    ResolvedCartesianTarget,
    ResolvedJointTarget,
    ResolvedMotionTarget,
    RigidTransform,
    TcpJogCommand,
    ToolContext,
)
from unilab_robot_contracts.geometry import quaternion_multiply

from ._support import validate_completion_receipt
from .moveit import MoveGroupPort


class MoveItCommissioningClientPort(MoveGroupPort, Protocol):
    """MoveIt 调试除执行外还必须提供新鲜关节和 TCP 状态。"""

    def read_commissioning_state(self) -> Mapping[str, Any]:
        """返回控制器可观测的连接、空闲、关节和 TCP 状态。"""

    def apply_tool_context(self, tool_context: ToolContext) -> Mapping[str, Any]:
        """更新 PlanningScene 并返回摘要/代次确认。"""


class MoveItCommissioningAdapter:
    """把统一维护命令解析成六轴 MoveIt 运动，不依赖 RViz。"""

    def __init__(
        self,
        *,
        port: MoveItCommissioningClientPort,
        model: Any,
        targets: Mapping[str, ResolvedMotionTarget],
        point_set_revision: str,
        hardware_profile_digest: str,
        tool_context_digest: str,
        commissioning_velocity_limit: float = 0.25,
        commissioning_acceleration_limit: float = 0.25,
        joint_completion_tolerance_si: float = 0.002,
        joint_readback_timeout_s: float = 0.5,
        group_name: str | None = None,
    ) -> None:
        """冻结 exact 型号、点位版本、硬件配置和工具上下文。"""

        if not point_set_revision.strip() or not hardware_profile_digest.strip():
            raise ValueError("MoveIt 调试必须绑定点位版本和 HardwareProfile")
        if not tool_context_digest.strip():
            raise ValueError("MoveIt 调试必须绑定 ToolContext digest")
        normalized: dict[str, ResolvedMotionTarget] = {}
        for target_ref, target in targets.items():
            if target_ref != target.target_ref:
                raise ValueError(f"MoveIt 调试 target_ref 索引漂移: {target_ref}")
            normalized[target_ref] = target
        self.port = port
        self.model = model
        self.targets = normalized
        self.point_set_revision = point_set_revision
        self.hardware_profile_digest = hardware_profile_digest
        self.tool_context_digest = tool_context_digest
        self.commissioning_velocity_limit = float(commissioning_velocity_limit)
        self.commissioning_acceleration_limit = float(commissioning_acceleration_limit)
        self.joint_completion_tolerance_si = float(joint_completion_tolerance_si)
        if not 0.0 < self.commissioning_velocity_limit <= 0.30:
            raise ValueError("HardwareProfile 调试速度上限必须位于 (0, 0.30]")
        if not 0.0 < self.commissioning_acceleration_limit <= 0.30:
            raise ValueError("HardwareProfile 调试加速度上限必须位于 (0, 0.30]")
        if not 0.0 < self.joint_completion_tolerance_si <= 0.01:
            raise ValueError("HardwareProfile 调试关节完成容差必须位于 (0, 0.01]")
        self.joint_readback_timeout_s = float(joint_readback_timeout_s)
        if not 0.0 <= self.joint_readback_timeout_s <= 2.0:
            raise ValueError("joint_jog 读回等待必须位于 [0, 2] 秒")
        self.group_name = group_name or str(model.planning_group)
        self._fingerprints: dict[str, str] = {}
        self._results: dict[str, CommandResult] = {}
        self._fenced_command_ids: set[str] = set()

    def activate_tool_context(self, tool_context: ToolContext) -> None:
        """仅在空闲时更新 TCP/碰撞模型，并验证 PlanningScene 回读。"""

        snapshot = self.commissioning_snapshot()
        if (
            not snapshot.is_fresh()
            or snapshot.online is not True
            or snapshot.idle is not True
            or snapshot.execution_fenced
        ):
            raise RuntimeError("MoveIt 非空闲或存在 Fence，禁止更新 ToolContext")
        receipt = self.port.apply_tool_context(tool_context)
        if (
            receipt.get("applied") is not True
            or str(receipt.get("tool_context_digest", "")) != tool_context.digest
            or int(receipt.get("attachment_generation", 0))
            != tool_context.attachment_generation
        ):
            raise RuntimeError("PlanningScene 未确认同一 ToolContext 摘要与附着代次")
        self.tool_context_digest = tool_context.digest

    @property
    def commissioning_capabilities(self) -> CommissioningCapabilities:
        """声明当前实现支持的封闭维护能力。"""

        return CommissioningCapabilities(
            move_target=True,
            move_pose=True,
            tcp_jog=True,
            joint_jog=True,
            controlled_stop=True,
        )

    @property
    def commissioning_target_revision(self) -> str:
        """返回当前活动 PointSet 的不可变版本。"""

        return self.point_set_revision

    def replace_point_set(
        self,
        targets: Mapping[str, ResolvedMotionTarget],
        point_set_revision: str,
    ) -> None:
        """热替换活动 PointSet，不重建 MoveIt 客户端。"""

        revision = str(point_set_revision).strip()
        if not revision:
            raise ValueError("MoveIt 调试必须绑定点位版本")
        normalized: dict[str, ResolvedMotionTarget] = {}
        for target_ref, target in targets.items():
            if target_ref != target.target_ref:
                raise ValueError(f"MoveIt 调试 target_ref 索引漂移: {target_ref}")
            normalized[target_ref] = target
        self.targets = normalized
        self.point_set_revision = revision

    def commissioning_snapshot(self) -> CommissioningSnapshot:
        """把 MoveGroup 控制器状态规范化为统一调试快照。"""

        raw = self.port.read_commissioning_state()
        joints = raw.get("joint_positions")
        joint_positions = None
        if isinstance(joints, Sequence) and not isinstance(joints, (str, bytes)):
            values = tuple(float(value) for value in joints)
            if len(values) != len(self.model.joint_specs):
                raise ValueError("MoveIt 调试状态关节数量与 exact Arm 型号不一致")
            joint_positions = tuple(
                CommissioningJointPosition(spec.name, value)
                for spec, value in zip(self.model.joint_specs, values, strict=True)
            )
        tcp_pose = _cartesian_pose(raw.get("tcp_pose"))
        known = (
            raw.get("online") is not None
            and raw.get("idle") is not None
            and joint_positions is not None
            and tcp_pose is not None
        )
        return CommissioningSnapshot(
            state=ObservationState.KNOWN if known else ObservationState.UNKNOWN,
            observed_at=float(raw.get("observed_at", time.time())),
            max_age_s=float(raw.get("max_age_s", 0.5)),
            source=str(raw.get("source", "moveit:commissioning")),
            online=(bool(raw["online"]) if raw.get("online") is not None else None),
            idle=(bool(raw["idle"]) if raw.get("idle") is not None else None),
            active_command_id=(
                str(raw["active_command_id"])
                if raw.get("active_command_id") is not None
                else None
            ),
            execution_fenced=(
                bool(raw.get("execution_fenced", False))
                or bool(self._fenced_command_ids)
            ),
            joint_positions=joint_positions,
            tcp_pose=tcp_pose,
        )

    def execute_commissioning(self, command: CommissioningCommand) -> CommandResult:
        """执行一个低速维护命令，未知结果绝不自动重放。"""

        existing = self._results.get(command.command_id)
        fingerprint = command.fingerprint()
        if existing is not None:
            if self._fingerprints[command.command_id] != fingerprint:
                return CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    "相同 command_id 的维护命令内容冲突",
                )
            return existing
        rejection = self._pre_dispatch_rejection(command)
        if rejection is not None:
            self._remember(command, rejection)
            return rejection
        if isinstance(command, ControlledStopCommand):
            confirmed = self.port.cancel(command.target_command_id)
            if confirmed:
                self._fenced_command_ids.discard(command.target_command_id)
            result = CommandResult(
                command.command_id,
                CommandState.CANCELED if confirmed else CommandState.EXECUTION_UNKNOWN,
                ("MoveIt 已确认维护运动停止" if confirmed else "MoveIt 停止确认不明"),
                {"target_command_id": command.target_command_id},
            )
            self._remember(command, result)
            return result
        rejection = self._command_rejection(command)
        if rejection is not None:
            self._remember(command, rejection)
            return rejection
        try:
            if isinstance(command, MoveTargetCommand):
                receipt = self._execute_move_target(command)
            elif isinstance(command, MovePoseCommand):
                receipt = self._execute_move_pose(command)
            elif isinstance(command, JointJogCommand):
                receipt = self._execute_joint_jog(command)
            elif isinstance(command, TcpJogCommand):
                receipt = self._execute_tcp_jog(command)
            else:
                result = CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    f"MoveIt 调试命令尚未接线: {command.kind.value}",
                )
                self._remember(command, result)
                return result
            result = _command_result_from_receipt(command, receipt)
        # 传输、规划或回执验证的任意异常都可能发生在物理派发之后，必须保守 UNKNOWN。
        except Exception as exc:  # noqa: BLE001
            result = CommandResult(
                command.command_id,
                CommandState.EXECUTION_UNKNOWN,
                f"MoveIt 调试派发结果不明: {exc}",
            )
        self._remember(command, result)
        return result

    def _command_rejection(self, command: CommissioningCommand) -> CommandResult | None:
        """校验所有无需物理派发即可确定的命令约束。"""

        message: str | None = None
        if isinstance(
            command,
            (MoveTargetCommand, MovePoseCommand, JointJogCommand, TcpJogCommand),
        ):
            if command.velocity_scale > self.commissioning_velocity_limit:
                message = "维护速度超过活动 HardwareProfile 上限"
            elif command.acceleration_scale > self.commissioning_acceleration_limit:
                message = "维护加速度超过活动 HardwareProfile 上限"
        if message is not None:
            return CommandResult(command.command_id, CommandState.REJECTED, message)
        if isinstance(command, MoveTargetCommand):
            if command.target_revision != self.point_set_revision:
                message = "move_target 点位版本与活动 PointSet 不一致"
            elif command.target_ref not in self.targets:
                message = "活动 PointSet 不包含 target_ref"
        elif isinstance(command, MovePoseCommand):
            if command.tool_context_digest != self.tool_context_digest:
                message = "move_pose ToolContext digest 与活动工具不一致"
            elif command.pose_input.frame_ref not in {
                "arm_base",
                self.model.base_frame,
            }:
                message = "move_pose 尚未配置到 arm_base 的静态变换"
        elif isinstance(command, JointJogCommand):
            joint_specs = {
                specification.name: specification
                for specification in self.model.joint_specs
            }
            specification = joint_specs.get(command.joint_ref)
            if specification is None:
                message = "joint_jog joint_ref 不属于 exact Arm 型号"
            else:
                snapshot = self.commissioning_snapshot()
                if snapshot.joint_positions is None:
                    message = "joint_jog 缺少完整关节状态"
                else:
                    current = {
                        item.joint_ref: item.position_si
                        for item in snapshot.joint_positions
                    }[command.joint_ref]
                    target = current + command.direction.sign * command.step_si
                    if specification.lower is not None and target < specification.lower:
                        message = "joint_jog 目标低于型号关节限位"
                    elif (
                        specification.upper is not None and target > specification.upper
                    ):
                        message = "joint_jog 目标高于型号关节限位"
        elif isinstance(command, TcpJogCommand):
            if self.commissioning_snapshot().tcp_pose is None:
                message = "tcp_jog 缺少当前 TCP 位姿"
        else:
            message = f"MoveIt 调试命令尚未接线: {command.kind.value}"
        if message is None:
            return None
        return CommandResult(command.command_id, CommandState.REJECTED, message)

    def _execute_move_target(self, command: MoveTargetCommand) -> Mapping[str, Any]:
        """校验活动点位版本并派发一个已解析目标。"""

        target = self.targets[command.target_ref]
        return self._execute_target(command, target)

    def _execute_joint_jog(self, command: JointJogCommand) -> Mapping[str, Any]:
        """从完整当前状态生成只改变一个关节的有限低速目标。"""

        snapshot = self.commissioning_snapshot()
        if snapshot.joint_positions is None:
            raise ValueError("joint_jog 缺少完整关节状态")
        current = tuple(item.position_si for item in snapshot.joint_positions)
        indices = {
            specification.name: index
            for index, specification in enumerate(self.model.joint_specs)
        }
        if command.joint_ref not in indices:
            raise ValueError("joint_jog joint_ref 不属于 exact Arm 型号")
        index = indices[command.joint_ref]
        target = list(current)
        target[index] += command.direction.sign * command.step_si
        specification = self.model.joint_specs[index]
        if specification.lower is not None and target[index] < specification.lower:
            raise ValueError("joint_jog 目标低于型号关节限位")
        if specification.upper is not None and target[index] > specification.upper:
            raise ValueError("joint_jog 目标高于型号关节限位")
        receipt = self.port.execute_joint_target(
            group_name=self.group_name,
            joint_names=self.model.joint_names,
            target=target,
            command_id=command.command_id,
            parameters=_commissioning_parameters(command, primitive="joint_ptp"),
        )
        if str(receipt.get("state", "")) != CommandState.SUCCEEDED.value:
            return receipt
        if self._joint_jog_reached(current, index, target):
            return receipt
        return {
            **dict(receipt),
            "state": CommandState.FAILED.value,
            "completed": False,
            "message": "joint_jog 完成见证与关节读回不一致",
        }

    def _joint_jog_reached(
        self,
        current: Sequence[float],
        moved_index: int,
        target: Sequence[float],
    ) -> bool:
        """MoveIt 成功后等到 /joint_states 跟上目标；仿真发布周期是 20ms。"""

        deadline = time.monotonic() + self.joint_readback_timeout_s
        while True:
            observed = self.commissioning_snapshot().joint_positions
            if observed is not None and _joint_jog_matches(
                self.model.joint_specs,
                current,
                tuple(item.position_si for item in observed),
                moved_index,
                target,
                self.joint_completion_tolerance_si,
            ):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def _execute_move_pose(self, command: MovePoseCommand) -> Mapping[str, Any]:
        """执行一次不写入 PointSet 的规范绝对 TCP 位姿。"""

        pose = command.pose_input.resolved_pose
        return self.port.execute_cartesian_target(
            group_name=self.group_name,
            frame_ref=self.model.base_frame,
            xyz_m=pose.xyz_m,
            orientation_xyzw=pose.orientation_xyzw,
            command_id=command.command_id,
            parameters=_commissioning_parameters(command, primitive="cartesian_linear"),
        )

    def _execute_tcp_jog(self, command: TcpJogCommand) -> Mapping[str, Any]:
        """从当前 TCP 构造一个有限平移或旋转目标。"""

        current = self.commissioning_snapshot().tcp_pose
        if current is None:
            raise ValueError("tcp_jog 缺少当前 TCP 位姿")
        signed_step = command.direction.sign * command.step_si
        if command.axis.rotational:
            axis_index = {"rx": 0, "ry": 1, "rz": 2}[command.axis.value]
            axis = tuple(1.0 if index == axis_index else 0.0 for index in range(3))
            delta = RigidTransform.from_axis_angle(axis, signed_step)
        else:
            axis_index = {"x": 0, "y": 1, "z": 2}[command.axis.value]
            translation = tuple(
                signed_step if index == axis_index else 0.0 for index in range(3)
            )
            delta = RigidTransform(translation, (0.0, 0.0, 0.0, 1.0))
        if command.frame_ref == "tool":
            target = current.transform.compose(delta)
        else:
            target = RigidTransform(
                tuple(
                    current.xyz_m[index] + delta.translation_m[index]
                    for index in range(3)
                ),
                quaternion_multiply(
                    delta.orientation_xyzw,
                    current.orientation_xyzw,
                ),
            )
        return self.port.execute_cartesian_target(
            group_name=self.group_name,
            frame_ref=self.model.base_frame,
            xyz_m=target.translation_m,
            orientation_xyzw=target.orientation_xyzw,
            command_id=command.command_id,
            parameters=_commissioning_parameters(command, primitive="cartesian_linear"),
        )

    def _pre_dispatch_rejection(
        self, command: CommissioningCommand
    ) -> CommandResult | None:
        """在任何物理派发前检查配置身份和新鲜空闲状态。"""

        if command.hardware_profile_digest != self.hardware_profile_digest:
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                "维护命令 HardwareProfile digest 不匹配",
            )
        snapshot = self.commissioning_snapshot()
        if isinstance(command, ControlledStopCommand):
            if (
                not snapshot.is_fresh()
                or snapshot.online is not True
                or snapshot.active_command_id != command.target_command_id
            ):
                return CommandResult(
                    command.command_id,
                    CommandState.REJECTED,
                    "controlled_stop 目标不是当前新鲜可观测的活动命令",
                )
            return None
        if (
            not snapshot.is_fresh()
            or snapshot.online is not True
            or snapshot.idle is not True
            or snapshot.execution_fenced
        ):
            return CommandResult(
                command.command_id,
                CommandState.REJECTED,
                "MoveIt 调试状态未知、过期、忙碌、离线或存在 Fence",
            )
        return None

    def _execute_target(
        self,
        command: MoveTargetCommand,
        target: ResolvedMotionTarget,
    ) -> Mapping[str, Any]:
        """把已解析目标派发到唯一 MoveGroup 端口。"""

        if isinstance(target, ResolvedJointTarget):
            return self.port.execute_joint_target(
                group_name=self.group_name,
                joint_names=self.model.joint_names,
                target=target.joint_positions,
                command_id=command.command_id,
                parameters=_commissioning_parameters(command, primitive="joint_ptp"),
            )
        if not isinstance(target, ResolvedCartesianTarget):
            raise TypeError("MoveIt 调试只接受已解析运动目标")
        return self.port.execute_cartesian_target(
            group_name=self.group_name,
            frame_ref=target.pose.frame_ref,
            xyz_m=target.pose.xyz_m,
            orientation_xyzw=target.pose.orientation_xyzw,
            command_id=command.command_id,
            parameters=_commissioning_parameters(command, primitive="cartesian_linear"),
        )

    def _remember(self, command: CommissioningCommand, result: CommandResult) -> None:
        """保存进程内幂等身份和结果；UNKNOWN 也不得重新派发。"""

        self._fingerprints[command.command_id] = command.fingerprint()
        self._results[command.command_id] = result
        if result.state is CommandState.EXECUTION_UNKNOWN:
            self._fenced_command_ids.add(command.command_id)


def _revolute_joint_error(actual: float, expected: float, specification: Any) -> float:
    """最短角误差：±2π 行程或 continuous 关节的读回可能折到对侧。"""

    linear = abs(actual - expected)
    joint_type = getattr(specification, "joint_type", None)
    lower = getattr(specification, "lower", None)
    upper = getattr(specification, "upper", None)
    wraps = getattr(joint_type, "value", joint_type) == "continuous" or (
        lower is not None
        and upper is not None
        and (upper - lower) >= 2.0 * math.pi - 1e-3
    )
    if not wraps:
        return linear
    wrapped = abs((actual - expected + math.pi) % (2.0 * math.pi) - math.pi)
    return min(linear, wrapped)


def _joint_jog_matches(
    specifications: Sequence[Any],
    before: Sequence[float],
    after: Sequence[float],
    moved_index: int,
    target: Sequence[float],
    tolerance: float,
) -> bool:
    """验证只有被点动关节到达目标，其余关节仍在容差内。"""

    for index, (previous, actual) in enumerate(zip(before, after, strict=True)):
        expected = target[index] if index == moved_index else previous
        if _revolute_joint_error(actual, expected, specifications[index]) > tolerance:
            return False
    return True


def _command_result_from_receipt(
    command: CommissioningCommand, receipt: Mapping[str, Any]
) -> CommandResult:
    """把 MoveIt 已知终态回执投影为命令结果；歧义回执仍抛给 UNKNOWN 路径。"""

    if str(receipt.get("command_id", "")) != command.command_id:
        raise ValueError("MoveIt commissioning 回执 command_id 不匹配")
    state = str(receipt.get("state", ""))
    if state == CommandState.FAILED.value:
        message = str(receipt.get("message") or "").strip() or "MoveIt 返回失败终态"
        return CommandResult(
            command.command_id,
            CommandState.FAILED,
            message,
            dict(receipt),
        )
    validate_completion_receipt(
        receipt,
        command_id=command.command_id,
        source="MoveIt commissioning",
    )
    return CommandResult(
        command.command_id,
        CommandState.SUCCEEDED,
        "MoveIt 调试运动完成",
        receipt,
    )


def _cartesian_pose(value: object) -> CartesianPose | None:
    """把端口映射转换为统一绝对 TCP 位姿。"""

    if not isinstance(value, Mapping):
        return None
    xyz = tuple(float(item) for item in value.get("xyz_m", ()))
    orientation = tuple(float(item) for item in value.get("orientation_xyzw", ()))
    if len(xyz) != 3 or len(orientation) != 4:
        return None
    return CartesianPose(
        str(value.get("frame_ref", "")),
        xyz,
        orientation,
    )


def _commissioning_parameters(
    command: MoveTargetCommand | MovePoseCommand | JointJogCommand | TcpJogCommand,
    *,
    primitive: str,
) -> Mapping[str, Any]:
    """把维护命令展开为 MoveIt 唯一接受的冻结运动参数。"""

    return {
        "motion_profile_ref": command.motion_profile_ref,
        "primitive": primitive,
        "velocity_scale": command.velocity_scale,
        "acceleration_scale": command.acceleration_scale,
        "position_tolerance_m": 0.001,
        "orientation_tolerance_rad": 0.01,
        "collision_check": True,
        "cartesian_path": primitive == "cartesian_linear",
        "commissioning_kind": command.kind.value,
    }


__all__ = ["MoveItCommissioningAdapter", "MoveItCommissioningClientPort"]
