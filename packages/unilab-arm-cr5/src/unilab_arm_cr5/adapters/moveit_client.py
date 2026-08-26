"""把既有 MoveIt2 客户端收敛为无 RViz 的 ``MoveGroupPort``。"""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from typing import Any

from unilab_robot_contracts import (
    CommandState,
    RigidTransform,
    ToolContext,
    apply_moveit_planning_budget,
)


_MOVEIT_ERROR_DETAILS = {
    -31: ("NO_IK_SOLUTION", "没有可用的逆运动学解"),
    -30: ("ABORT", "MoveIt 主动中止"),
    -29: ("CRASH", "MoveIt 进程异常退出"),
    -25: ("COMMUNICATION_FAILURE", "MoveIt 通信失败"),
    -24: ("SENSOR_INFO_STALE", "传感器信息过期"),
    -23: ("ROBOT_STATE_STALE", "机械臂状态过期"),
    -22: ("COLLISION_CHECKING_UNAVAILABLE", "碰撞检测不可用"),
    -21: ("FRAME_TRANSFORM_FAILURE", "坐标变换失败"),
    -19: ("INVALID_OBJECT_NAME", "碰撞对象名称无效"),
    -18: ("INVALID_LINK_NAME", "机械臂连杆名称无效"),
    -17: ("INVALID_ROBOT_STATE", "机械臂状态无效"),
    -16: ("INVALID_GOAL_CONSTRAINTS", "目标约束无效"),
    -15: ("INVALID_GROUP_NAME", "规划组名称无效"),
    -14: ("GOAL_CONSTRAINTS_VIOLATED", "目标状态违反约束"),
    -13: ("GOAL_VIOLATES_PATH_CONSTRAINTS", "目标状态违反路径约束"),
    -12: ("GOAL_IN_COLLISION", "目标状态碰撞"),
    -11: ("START_STATE_VIOLATES_PATH_CONSTRAINTS", "起始状态违反路径约束"),
    -10: ("START_STATE_IN_COLLISION", "起始状态碰撞"),
    -7: ("PREEMPTED", "规划或执行被抢占"),
    -6: ("TIMED_OUT", "规划或执行超时"),
    -5: ("UNABLE_TO_AQUIRE_SENSOR_DATA", "无法取得传感器数据"),
    -4: ("CONTROL_FAILED", "控制器执行失败"),
    -3: (
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
        "环境变化使运动计划失效",
    ),
    -2: ("INVALID_MOTION_PLAN", "运动计划无效"),
    -1: ("PLANNING_FAILED", "规划失败，MoveIt 未提供更细原因"),
    99999: ("FAILURE", "MoveIt 通用失败"),
}
_CARTESIAN_FRACTION_THRESHOLD = 1.0


class MoveIt2ClientPort:
    """只依赖 MoveIt action/service 客户端，不拥有 RViz 或可视化生命周期。"""

    def __init__(
        self,
        client: Any,
        *,
        qualified_joint_names: Sequence[str],
        collision_asset_resolver: Any = None,
    ) -> None:
        """注入 OS/ROS 节点创建的 MoveIt2 客户端和完全限定的六轴关节名。"""

        names = tuple(str(name) for name in qualified_joint_names)
        if len(names) != 6:
            raise ValueError("MoveIt2ClientPort 只接受六轴机械臂")
        self.client = apply_moveit_planning_budget(client)
        self.qualified_joint_names = names
        self._results: dict[str, Mapping[str, Any]] = {}
        self._active_command_id: str | None = None
        self._active_tool_context: ToolContext | None = None
        self._collision_asset_resolver = collision_asset_resolver

    def execute_joint_target(
        self,
        *,
        group_name: str,
        joint_names: Sequence[str],
        target: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """通过 ``move_action`` 规划并执行六轴目标，等待明确终态见证。"""

        if len(tuple(joint_names)) != 6 or len(tuple(target)) != 6:
            raise ValueError("MoveIt 目标与关节名必须均为六轴")
        self._apply_motion_profile(parameters)
        self._active_command_id = command_id
        try:
            self.client.move_to_configuration(
                joint_positions=[float(value) for value in target],
                joint_names=list(self.qualified_joint_names),
            )
            completed = bool(self.client.wait_until_executed())
        finally:
            self._active_command_id = None
        state = CommandState.SUCCEEDED if completed else CommandState.FAILED
        message, error_code, error_name, cartesian_fraction = _moveit_result_detail(
            self.client,
            completed,
            inspect_cartesian_plan=False,
        )
        result = {
            "command_id": command_id,
            "state": state.value,
            "completed": completed,
            "message": message,
            "group_name": group_name,
        }
        if error_code is not None:
            result["moveit_error_code"] = error_code
            result["moveit_error_name"] = error_name
        if cartesian_fraction is not None:
            result["cartesian_fraction"] = cartesian_fraction
        self._results[command_id] = result
        return result

    def execute_cartesian_target(
        self,
        *,
        group_name: str,
        frame_ref: str,
        xyz_m: Sequence[float],
        orientation_xyzw: Sequence[float],
        command_id: str,
        parameters: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """通过 ``move_to_pose`` 执行解析后的绝对 TCP 位姿。

        参数：目标必须已在 ``arm_base`` 中展开；``cartesian_path`` 只选择
        MoveIt 内部路径规划方式，不重新解释相对点。
        返回：精确绑定命令身份的完成见证。
        异常：坐标系或长度错误时在派发前抛出 ``ValueError``。
        """

        position = tuple(float(value) for value in xyz_m)
        orientation = tuple(float(value) for value in orientation_xyzw)
        if frame_ref != "arm_base":
            raise ValueError("MoveIt2ClientPort 只接受已归一化到 arm_base 的绝对目标")
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError("MoveIt 笛卡尔目标必须是 xyz[3] 与 quaternion[4]")
        mount_target = RigidTransform(position, orientation)
        if self._active_tool_context is not None:
            mount_target = mount_target.compose(
                self._active_tool_context.mount_to_tcp.inverse()
            )
        self._apply_motion_profile(parameters)
        self._active_command_id = command_id
        try:
            cartesian_path = bool(parameters.get("cartesian_path", False))
            self.client.move_to_pose(
                position=mount_target.translation_m,
                quat_xyzw=mount_target.orientation_xyzw,
                frame_id=None,
                cartesian=cartesian_path,
                cartesian_fraction_threshold=_CARTESIAN_FRACTION_THRESHOLD,
                tolerance_position=float(parameters.get("position_tolerance_m", 0.001)),
                tolerance_orientation=float(
                    parameters.get("orientation_tolerance_rad", 0.001)
                ),
            )
            completed = bool(self.client.wait_until_executed())
        finally:
            self._active_command_id = None
        state = CommandState.SUCCEEDED if completed else CommandState.FAILED
        message, error_code, error_name, cartesian_fraction = _moveit_result_detail(
            self.client,
            completed,
            inspect_cartesian_plan=cartesian_path,
        )
        result = {
            "command_id": command_id,
            "state": state.value,
            "completed": completed,
            "message": message,
            "group_name": group_name,
            "target_type": "cartesian_pose",
        }
        if error_code is not None:
            result["moveit_error_code"] = error_code
            result["moveit_error_name"] = error_name
        if cartesian_fraction is not None:
            result["cartesian_fraction"] = cartesian_fraction
        self._results[command_id] = result
        return result

    def _apply_motion_profile(self, parameters: Mapping[str, Any]) -> None:
        """校验并应用 D10 冻结的速度、加速度和规划原语。"""

        profile_ref = parameters.get("motion_profile_ref")
        if profile_ref is None:
            velocity = acceleration = 1.0
        else:
            required = {
                "primitive",
                "velocity_scale",
                "acceleration_scale",
                "position_tolerance_m",
                "orientation_tolerance_rad",
                "collision_check",
                "cartesian_path",
            }
            missing = required.difference(parameters)
            if missing:
                raise ValueError(
                    f"MotionProfile {profile_ref} 缺少冻结字段: {sorted(missing)}"
                )
            primitive = str(parameters["primitive"])
            if primitive not in {"joint_ptp", "cartesian_linear"}:
                raise ValueError(f"MoveIt 不支持 MotionPrimitive: {primitive}")
            if not isinstance(parameters["cartesian_path"], bool):
                raise TypeError("MotionProfile.cartesian_path 必须是布尔值")
            if parameters["cartesian_path"] != (primitive == "cartesian_linear"):
                raise ValueError("cartesian_path 与 MotionPrimitive 不一致")
            if not isinstance(parameters["collision_check"], bool):
                raise TypeError("MotionProfile.collision_check 必须是布尔值")
            velocity = float(parameters["velocity_scale"])
            acceleration = float(parameters["acceleration_scale"])
            if any(
                float(parameters[name]) <= 0.0
                for name in ("position_tolerance_m", "orientation_tolerance_rad")
            ):
                raise ValueError("MotionProfile 到位容差必须是正数")
        if not 0.0 < velocity <= 1.0 or not 0.0 < acceleration <= 1.0:
            raise ValueError("MoveIt 速度和加速度缩放必须位于 (0, 1]")
        self.client.max_velocity = velocity
        self.client.max_acceleration = acceleration

    def query_command(self, command_id: str) -> Mapping[str, Any] | None:
        """返回当前进程已取得的 MoveIt 完成见证，不猜测丢失结果。"""

        return self._results.get(command_id)

    def cancel(self, command_id: str) -> bool:
        """请求 MoveIt 停止；因现有客户端无停止确认，始终保持 UNKNOWN。"""

        del command_id
        self.client.cancel_execution()
        return False

    def read_commissioning_state(self) -> Mapping[str, Any]:
        """读取 MoveIt2 当前关节与 FK TCP；缺少源数据时返回未知快照。"""

        direct = getattr(self.client, "read_commissioning_state", None)
        if callable(direct):
            return direct()
        joint_state = getattr(self.client, "joint_state", None)
        if joint_state is None:
            return {
                "observed_at": 0.0,
                "max_age_s": 0.5,
                "source": "moveit2:joint_states",
                "online": None,
                "idle": None,
                "active_command_id": self._active_command_id,
                "execution_fenced": False,
            }
        names = tuple(str(name) for name in getattr(joint_state, "name", ()))
        positions = tuple(
            float(value) for value in getattr(joint_state, "position", ())
        )
        by_name = dict(zip(names, positions))
        try:
            ordered = [by_name[name] for name in self.qualified_joint_names]
        except KeyError:
            ordered = []
        pose_stamped = self.client.compute_fk(joint_state=joint_state)
        tcp_pose = None
        if pose_stamped is not None:
            pose = pose_stamped.pose
            observed_mount = RigidTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )
            observed_tcp = (
                observed_mount
                if self._active_tool_context is None
                else observed_mount.compose(self._active_tool_context.mount_to_tcp)
            )
            tcp_pose = {
                "frame_ref": "arm_base",
                "xyz_m": list(observed_tcp.translation_m),
                "orientation_xyzw": list(observed_tcp.orientation_xyzw),
            }
        # CommissioningSnapshot.is_fresh() 按接收时钟判断，不能用 ROS header stamp。
        # compute_fk 会阻塞；工位 URDF 变大后 FK 常超过 max_age_s=0.5s。
        received_at = time.time()
        state = self.client.query_state()
        state_name = str(getattr(state, "name", state)).lower()
        return {
            "observed_at": received_at,
            "max_age_s": 0.5,
            "source": "moveit2:joint_states+compute_fk",
            "online": bool(ordered and tcp_pose is not None),
            "idle": state_name == "idle",
            "active_command_id": self._active_command_id,
            "execution_fenced": False,
            "joint_positions": ordered or None,
            "tcp_pose": tcp_pose,
        }

    def apply_tool_context(self, tool_context: ToolContext) -> Mapping[str, Any]:
        """委托 OS MoveIt 客户端更新 TCP/碰撞模型；不支持时关闭失败。"""

        if not tool_context.planning_scene:
            raise ValueError("MoveIt ToolContext 缺少 PlanningScene 碰撞模型")
        apply = getattr(self.client, "apply_tool_context", None)
        if not callable(apply):
            raise TypeError("当前 MoveIt2 客户端未实现 ToolContext PlanningScene 更新")
        if self._collision_asset_resolver is None:
            receipt = apply(tool_context)
        else:
            receipt = apply(
                tool_context,
                collision_asset_resolver=self._collision_asset_resolver,
            )
        if not isinstance(receipt, Mapping):
            raise TypeError("MoveIt2 ToolContext 更新缺少结构化确认")
        if (
            receipt.get("applied") is True
            and str(receipt.get("tool_context_digest", "")) == tool_context.digest
            and int(receipt.get("attachment_generation", 0))
            == tool_context.attachment_generation
        ):
            self._active_tool_context = tool_context
        return receipt


def _moveit_result_detail(
    client: Any,
    completed: bool,
    *,
    inspect_cartesian_plan: bool,
) -> tuple[str, int | None, str | None, float | None]:
    """把 MoveIt 终态错误码翻译为可操作的规划/碰撞诊断。"""

    if completed:
        return "MoveIt 完成", None, None, None
    cartesian_fraction: float | None = None
    if inspect_cartesian_plan:
        fraction_getter = getattr(client, "get_last_cartesian_fraction", None)
        if callable(fraction_getter):
            raw_fraction = fraction_getter()
            if raw_fraction is not None:
                cartesian_fraction = float(raw_fraction)
        if (
            cartesian_fraction is not None
            and cartesian_fraction < _CARTESIAN_FRACTION_THRESHOLD
        ):
            percentage = cartesian_fraction * 100.0
            return (
                "MoveIt 笛卡尔规划路径覆盖率不足: "
                f"{percentage:.6f}%（要求 100%）；可能是目标不可达、"
                "起始状态/目标状态碰撞或路径中途碰撞",
                None,
                None,
                cartesian_fraction,
            )
    getter = None
    if inspect_cartesian_plan:
        getter = getattr(client, "get_last_planning_error_code", None)
    if not callable(getter):
        getter = getattr(client, "get_last_execution_error_code", None)
    if not callable(getter):
        return (
            "MoveIt 返回失败终态（未提供错误码）",
            None,
            None,
            cartesian_fraction,
        )
    raw_error = getter()
    raw_code = getattr(raw_error, "val", raw_error)
    try:
        code = int(raw_code)
    except (TypeError, ValueError):
        return (
            "MoveIt 返回失败终态（错误码不可读）",
            None,
            None,
            cartesian_fraction,
        )
    name, description = _MOVEIT_ERROR_DETAILS.get(
        code,
        ("UNKNOWN_MOVEIT_ERROR", "未识别的 MoveIt 错误码"),
    )
    return (
        f"MoveIt 返回失败终态: {name} ({code})，{description}",
        code,
        name,
        cartesian_fraction,
    )
