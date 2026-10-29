"""MoveIt ``move_group`` 服务后端：只暴露 ``PlanRequest``/``PlanResult``。

本模块是 ``MoveItPlanner`` 的第二种真实实现。``MoveItPlanner`` 需要
``moveit.planning``（MoveItPy）Python 模块，而本机的 RoboStack 发行版只有
``move_group`` 可执行文件；两者解耦后，设备包只依赖
``unilab_robot_contracts.planning``，不必知道后端是 Python 绑定还是服务。

数据面：

* ``/compute_ik``：先判定位姿目标本身有没有逆解（``ik_no_solution``），
  把「够不着」与「有解但会撞」分开；
* ``/check_state_validity``：判定 IK 解与沿轨迹的离散采样是否碰撞；
* ``/apply_planning_scene``：装载 ``CollisionWorld`` 与附着体；
* ``/plan_kinematic_path``：真正的 OMPL/Pilz 规划。

失败不伪造：导入失败或无服务时返回 ``status="invalid"`` 且
``failure_category="planner_unavailable"``；MoveIt 的错误码按
``MoveItErrorCodes`` 逐项映射，原始码保留在 ``diagnostics``。本后端不创建
控制器、不调用执行、不连接硬件。
"""

from __future__ import annotations

import importlib
import importlib.metadata
import math
import time
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

from unilab_robot_contracts.planning import (
    CartesianGoal,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    PlanRequest,
    PlanResult,
    PlanStatus,
    PlanWaypoint,
    PlannerIdentity,
    RailAxis,
    RobotModel,
)

from .planning_timing import PlanningTiming, timed_stage
from .moveit_error_codes import MOVEIT_ERROR_CODES, describe_moveit_error
from .moveit_ompl_config import OMPL_PLANNING_PLUGIN, require_ompl_planning_plugin

_PLANNER_NAME = "MoveIt move_group"
_IMPLEMENTATION = "move_group_ros2_services"
_PLAN_SERVICE = "/plan_kinematic_path"
_SCENE_SERVICE = "/apply_planning_scene"
_VALIDITY_SERVICE = "/check_state_validity"
_IK_SERVICE = "/compute_ik"

#: ``MoveItErrorCodes`` 的显式映射见 ``moveit_error_codes``：那是唯一的权威表
#: （-2 invalid_motion_plan、-16 invalid_goal_constraints、-31 no_ik_solution、
#: -1 failure……）。这里保留名字 ``_ERROR_CODES`` 只是为了不打断既有引用。
_ERROR_CODES = MOVEIT_ERROR_CODES

#: move_group 侧的瞬时错误：同一请求重发即可恢复，第一次的原始码仍记录在
#: ``diagnostics.first_attempt``。``-26`` 是 ``START_STATE_INVALID``（场景刚更新
#: 时常见的过期起始态），``-29`` 是 ``CRASH``，``-7`` 是 ``PREEMPTED``。
_TRANSIENT_ERROR_CODES = frozenset({-26, -29, -7})


class MoveGroupUnavailableError(RuntimeError):
    """当前解释器没有 rclpy/moveit_msgs，或 ``move_group`` 服务不可用。"""


@dataclass(frozen=True, slots=True)
class MoveGroupPlannerConfig:
    """``move_group`` 后端的连接与规划调参。

    ``domain_id`` 必须在进程启动前生效；这里只校验环境变量，不改写运行中的
    DDS 配置。``joint_acceleration_rad_s2`` 是**显式仿真假设**（URDF 没有加速度
    字段），由调用方写进规划参数并在报告里标注，不冒充厂家限位。
    """

    node_name: str = "unilab_move_group_planner"
    pipeline: str = "ompl"
    planner_id: str = "RRTConnectkConfigDefault"
    planning_time_s: float = 10.0
    planning_attempts: int = 1
    # MoveIt 2.5.9 may call simplifyMax when solve overruns its budget.
    # Keep simplification a separate explicitly bounded, revalidated stage.
    simplify_solutions: bool = False
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    max_joint_sample_step_rad: float = 0.05
    collision_contacts: int = 30
    ik_timeout_s: float = 0.2
    service_timeout_s: float = 120.0
    domain_id: int | None = None
    service_wait_s: float = 30.0
    joint_acceleration_rad_s2: float = 0.5
    #: OMPL 的路径离散校验粒度（状态空间范围的分数）。默认 0.005 对 6 轴臂约合
    #: 0.06 rad，比本后端的 0.05 rad 复采样还粗，会出现「MoveIt 判成功、复采样
    #: 判碰撞」的假象；这里收紧到 0.001（约 0.0125 rad）。
    longest_valid_segment_fraction: float = 0.001
    #: 导轨速度/加速度限位是**显式仿真假设**：URDF 只给 velocity=0.05 m/s、
    #: 没有加速度字段；TOTG 需要加速度才能给出非零时长。
    rail_velocity_m_s: float = 0.05
    rail_acceleration_m_s2: float = 0.25
    # RoboStack MoveIt 2.5.9 evaluates rotation vectors in desired-link axes.
    # Newer builds may use reference-frame axes; select only after a native probe.
    native_orientation_error_frame: str = "desired_link"

    def __post_init__(self) -> None:
        if self.native_orientation_error_frame not in ("desired_link", "reference"):
            raise ValueError("Unknown native orientation error frame")
        if not str(self.node_name).strip():
            raise ValueError("node_name 不能为空")
        if not str(self.pipeline).strip() or not str(self.planner_id).strip():
            raise ValueError("pipeline/planner_id 不能为空")
        for attr in ("planning_time_s", "velocity_scaling", "acceleration_scaling",
                     "max_joint_sample_step_rad", "ik_timeout_s", "service_timeout_s",
                     "service_wait_s", "joint_acceleration_rad_s2",
                     "longest_valid_segment_fraction", "rail_velocity_m_s",
                     "rail_acceleration_m_s2"):
            value = float(getattr(self, attr))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{attr} 必须是正有限数")
            object.__setattr__(self, attr, value)
        for attr in ("planning_attempts", "collision_contacts"):
            value = getattr(self, attr)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{attr} 必须是正整数")
        if self.domain_id is not None:
            value = self.domain_id
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError("domain_id 必须是非负整数")


def _distribution_version(*names: str) -> str:
    for name in names:
        try:
            return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
        except Exception:  # pragma: no cover - 损坏的元数据随环境而变
            continue
    return "unknown"


def _module_version(module_name: str) -> str:
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return "unknown"
    value = getattr(module, "__version__", None)
    return "unknown" if value is None else str(value)


def _ament_package_version(package: str) -> str:
    """从 ``ament`` 索引读 ROS 包 ``package.xml`` 的版本（conda/ROS 安装没有 pip 元数据）。"""

    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        return "unknown"
    try:
        share = get_package_share_directory(package)
    except Exception:
        return "unknown"
    import re

    from pathlib import Path

    try:
        text = (Path(share) / "package.xml").read_text(encoding="utf-8")
    except OSError:
        return "unknown"
    match = re.search(r"<version>\s*([^<]+)</version>", text)
    return match.group(1).strip() if match else "unknown"


def _version(first: str, *fallbacks: str) -> str:
    """按顺序返回第一个真实版本号，全部缺失时写 ``unknown``。"""

    for candidate in (first, *fallbacks):
        if candidate and candidate != "unknown":
            return candidate
    return "unknown"


def detect_planner_identity() -> PlannerIdentity:
    """不初始化 ROS 的规划器身份；版本逐项探测，缺失写 ``unknown``。"""

    moveit = _version(
        _distribution_version("moveit", "moveit-core", "moveit_ros_planning"),
        _ament_package_version("moveit_ros_planning"),
        _ament_package_version("moveit_core"),
    )
    return PlannerIdentity(
        name=_PLANNER_NAME,
        implementation=_IMPLEMENTATION,
        version=moveit,
        engine_versions={
            "moveit": moveit,
            "moveit_msgs": _version(_distribution_version("moveit_msgs"),
                                    _ament_package_version("moveit_msgs")),
            "rclpy": _version(_distribution_version("rclpy", "ros-rclpy"),
                              _ament_package_version("rclpy")),
            "ompl": _version(_module_version("ompl"), _ament_package_version("ompl")),
            "transport": "ros2_services",
        },
    )


def build_move_group_parameters(
    model: RobotModel,
    config: MoveGroupPlannerConfig | None = None,
    *,
    rail_axis: RailAxis | None = None,
) -> dict[str, Any]:
    """把 ``RobotModel`` 翻译成 ``move_group`` 的 ``ros__parameters`` 内容。

    返回参数**内容**；调用方负责写 ``/**/ros__parameters`` 前缀并在启动前把
    ``robot_description`` 装进参数服务器。加速度限位是显式仿真假设（URDF 没有
    加速度字段），写进 ``robot_description_planning`` 供 TOTG 使用。

    声明 ``rail_axis`` 时额外写入导轨关节的速度/加速度限位，并为 SRDF 中只含
    该关节的规划组补一份 OMPL 配置；导轨仍然是独立轴，不并入六轴组。

    时间参数化使用本机 2.5.9 实际提供的 **request** adapter
    ``AddTimeOptimalParameterization``（TOTG）。响应侧
    ``ValidateSolution`` 插件在 RoboStack 构建里没有随包发布，不能写进
    ``response_adapters``，否则 ``move_group`` 启动即失败。
    """

    settings = config if config is not None else MoveGroupPlannerConfig()
    group = model.planning_group
    limits: dict[str, dict[str, Any]] = {}
    for index, name in enumerate(model.joint_names):
        bounds = model.joint_limits[index] if model.joint_limits else None
        entry: dict[str, Any] = {
            "has_position_limits": bounds is not None,
            "has_velocity_limits": True,
            "max_velocity": math.pi,
            "has_acceleration_limits": True,
            "max_acceleration": settings.joint_acceleration_rad_s2,
            "has_deceleration_limits": True,
            "max_deceleration": -settings.joint_acceleration_rad_s2,
        }
        if bounds is not None:
            entry["min_position"] = bounds[0]
            entry["max_position"] = bounds[1]
        limits[name] = entry
    rail_group: str | None = None
    if rail_axis is not None:
        limits[rail_axis.axis_id] = {
            "has_position_limits": True,
            "min_position": rail_axis.limits_m[0],
            "max_position": rail_axis.limits_m[1],
            "has_velocity_limits": True,
            "max_velocity": settings.rail_velocity_m_s,
            "has_acceleration_limits": True,
            "max_acceleration": settings.rail_acceleration_m_s2,
            "has_deceleration_limits": True,
            "max_deceleration": -settings.rail_acceleration_m_s2,
        }
        rail_group = rail_planning_group(model, rail_axis.axis_id)
    ompl_groups: dict[str, Any] = {
        group: {
            "default_planner_config": settings.planner_id,
            "planner_configs": [settings.planner_id],
            "longest_valid_segment_fraction": settings.longest_valid_segment_fraction,
            "enforce_joint_model_state_space": True,
        }
    }
    if rail_group is not None:
        ompl_groups[rail_group] = {
            "default_planner_config": settings.planner_id,
            "planner_configs": [settings.planner_id],
            "longest_valid_segment_fraction": settings.longest_valid_segment_fraction,
            "enforce_joint_model_state_space": True,
        }
    ompl_parameters: dict[str, Any] = {
        # MoveIt 2.5.9 的 planning_pipeline 仍先读单数 `planning_plugin`；
        # 只给复数 `planning_plugins` 会静默回落到 CHOMP（它不支持位姿目标）。
        "planning_plugin": OMPL_PLANNING_PLUGIN,
        "planning_plugins": [OMPL_PLANNING_PLUGIN],
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
        "planner_configs": {
            settings.planner_id: {
                "type": "geometric::RRTConnect", "range": 0.0,
                # Humble whitelists group parameters; simplify_solutions is
                # only loaded through the per-planner configuration namespace.
                # Humble uses boost::lexical_cast<bool> on value_to_string().
                # Integer 0/1 is portable; ROS boolean "false"/"true" throws.
                "simplify_solutions": int(settings.simplify_solutions),
            },
        },
        **ompl_groups,
    }
    # 构造期即拒绝任何可能被 MoveIt 静默换成 CHOMP 的形态。
    require_ompl_planning_plugin(ompl_parameters)
    return {
        "robot_description": model.urdf_xml,
        "robot_description_semantic": model.srdf_xml,
        "robot_description_kinematics": dict(model.kinematics),
        "robot_description_planning": {
            "default_acceleration_scaling_factor": settings.acceleration_scaling,
            "joint_limits": limits,
            "cartesian_limits": {
                "max_trans_vel": 0.15,
                "max_trans_acc": 0.3,
                "max_trans_dec": -0.3,
                "max_rot_vel": 0.5,
            },
        },
        # 调用方显式选择已验证的管线。Windows Pilz 依赖 Library/lib 中的
        # tf2_eigen_kdl.dll；使用 ros_environment.move_group_environment 提供搜索路径。
        "planning_pipelines": [settings.pipeline],
        "default_planning_pipeline": settings.pipeline,
        "ompl": ompl_parameters,
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
        },
        "plan_request_params": {
            "planning_attempts": settings.planning_attempts,
            "planning_pipeline": settings.pipeline,
            "planner_id": settings.planner_id,
            "max_velocity_scaling_factor": settings.velocity_scaling,
            "max_acceleration_scaling_factor": settings.acceleration_scaling,
            "planning_time": settings.planning_time_s,
        },
        "trajectory_execution": {"manage_controllers": False},
        "use_sim_time": False,
        "capabilities": "",
    }


def rail_planning_group(model: RobotModel, rail_joint: str) -> str | None:
    """在 SRDF 中查找只包含该导轨关节的规划组。

    合同把导轨定义为独立轴；导轨段的规划组必须恰好是 ``{rail_joint}``。
    找不到时返回 ``None``，调用方必须显式拒绝，不得把导轨并进六轴组或
    静默按导轨 0 规划。
    """

    try:
        root = ET.fromstring(model.srdf_xml)
    except ET.ParseError:
        return None
    for group in root.findall("group"):
        joints = {
            item.get("name") for item in group.findall("joint") if item.get("name")
        }
        if joints == {rail_joint}:
            name = group.get("name")
            if name:
                return str(name)
    return None


def _matrix_to_native(matrix: Sequence[Sequence[float]]) -> Any:
    """合同 4x4 矩阵转 ``geometry_msgs/Pose``（不依赖 scipy）。"""

    from geometry_msgs.msg import Pose

    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    if len(rows) != 4 or any(len(row) != 4 for row in rows):
        raise ValueError("pose 必须是 4x4 矩阵")
    rotation = tuple(tuple(rows[row][column] for column in range(3)) for row in range(3))
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qx = (rotation[2][1] - rotation[1][2]) / scale
        qy = (rotation[0][2] - rotation[2][0]) / scale
        qz = (rotation[1][0] - rotation[0][1]) / scale
        qw = scale / 4.0
    else:
        index = max(range(3), key=lambda item: rotation[item][item])
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = math.sqrt(max(
            0.0,
            1.0 + rotation[index][index] - rotation[next_index][next_index]
            - rotation[last_index][last_index])) * 2.0
        quaternion = [0.0, 0.0, 0.0, 0.0]
        quaternion[index] = scale / 4.0
        quaternion[next_index] = (rotation[next_index][index] + rotation[index][next_index]) / scale
        quaternion[last_index] = (rotation[last_index][index] + rotation[index][last_index]) / scale
        quaternion[3] = (rotation[last_index][next_index] - rotation[next_index][last_index]) / scale
        qx, qy, qz, qw = quaternion
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (rows[0][3], rows[1][3], rows[2][3])
    pose.orientation.x, pose.orientation.y = qx, qy
    pose.orientation.z, pose.orientation.w = qz, qw
    return pose


def _identity_pose() -> Any:
    return _matrix_to_native((
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _geometry_to_native(geometry: CollisionGeometry) -> tuple[Any, Any]:
    """返回 ``(mesh, primitive)``；文件 mesh 必须由调用方先读成内存网格。"""

    from shape_msgs.msg import SolidPrimitive

    if geometry.kind == "box":
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(geometry.size_m or ())
        return None, primitive
    if geometry.kind == "cylinder":
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(geometry.length_m), float(geometry.radius_m)]
        return None, primitive
    from geometry_msgs.msg import Point
    from shape_msgs.msg import Mesh, MeshTriangle

    if geometry.vertices is None or geometry.triangles is None:
        raise ValueError("move_group 后端只支持内存 mesh；文件 mesh 请先读成顶点/三角面")
    scale = geometry.mesh_scale
    mesh = Mesh()
    mesh.vertices = [Point(x=vertex[0] * scale[0], y=vertex[1] * scale[1],
                           z=vertex[2] * scale[2]) for vertex in geometry.vertices]
    mesh.triangles = [MeshTriangle(vertex_indices=[int(index) for index in triangle])
                      for triangle in geometry.triangles]
    return mesh, None


def _object_to_native(obj: CollisionObject) -> Any:
    from moveit_msgs.msg import CollisionObject as NativeCollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle

    native = NativeCollisionObject()
    native.id = obj.object_id
    native.header.frame_id = "world"
    native.pose = _matrix_to_native(obj.world_pose)
    native.operation = NativeCollisionObject.ADD
    for index, geometry in enumerate(obj.geometry):
        mesh, primitive = _geometry_to_native(geometry)
        pose = (_matrix_to_native(obj.geometry_poses[index])
                if obj.geometry_poses else _identity_pose())
        if mesh is not None:
            native.meshes.append(mesh)
            native.mesh_poses.append(pose)
        elif primitive is not None:
            native.primitives.append(primitive)
            native.primitive_poses.append(pose)
    if not native.meshes and not native.primitives:
        raise ValueError(f"对象 {obj.object_id} 没有可装载几何")
    return native


@dataclass
class _Session:
    """延迟建立的 ROS 会话；只在第一次使用时导入 rclpy。"""

    rclpy: Any = None
    node: Any = None
    clients: dict[str, Any] = field(default_factory=dict)
    messages: dict[str, Any] = field(default_factory=dict)


class MoveGroupPlanner:
    """``move_group`` 服务后端；构造时不导入 ROS，第一次使用时才连接。

    一个实例可连续规划多个请求；碰撞世界只在 ``CollisionWorld`` 摘要或附着体
    变化时重装。``plan`` 不做任何执行，也不读磁盘上的历史轨迹。
    """

    def __init__(self, config: MoveGroupPlannerConfig | None = None) -> None:
        self.config = config if config is not None else MoveGroupPlannerConfig()
        self._session = _Session()
        self._timing = PlanningTiming()
        self._ready = False
        self._loaded_scene_key: tuple[Any, ...] | None = None
        self._scene_receipt: str | None = None
        self._loaded_ids: set[str] = set()
        self._attached_ids: set[str] = set()
        self._allowed_pairs: set[tuple[str, str]] = set()
        self._acm_original: dict[tuple[str, str], bool] = {}
        self._loaded_objects: dict[str, CollisionObject] = {}
        self._loaded_attachment_specs = {}
        # 碰撞世界摘要要对全部三角面做规范 JSON，几十万面时单次就要数秒；
        # 这里按对象身份缓存，避免每条请求重复计算。
        self._world_digests: dict[int, tuple[CollisionWorld, str]] = {}
        self.planning_identity = detect_planner_identity()

    # ------------------------------------------------------------------ 会话
    @property
    def identity(self) -> PlannerIdentity:
        return self.planning_identity

    def performance_snapshot(self) -> dict[str, Any]:
        """返回会话累计阶段计时；RPC 包含通信与服务执行，非纯 FCL 时间。"""
        return {"schema": "unilab.planning-timing/v1",
                "stages": self._timing.snapshot(),
                "stage_times_are_inclusive": True,
                "rpc_includes_transport_and_server": True,
                "pure_fcl_s": None}

    def _import_native(self) -> None:
        if self._session.messages:
            return
        try:
            import rclpy
            from geometry_msgs.msg import Point, Pose, PoseStamped
            from moveit_msgs.msg import AttachedCollisionObject, BoundingVolume
            from moveit_msgs.msg import CollisionObject as NativeObject
            from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
            from moveit_msgs.msg import PlanningScene, PositionConstraint, RobotState
            from moveit_msgs.msg import PlanningSceneComponents, AllowedCollisionEntry
            from moveit_msgs.srv import (
                ApplyPlanningScene,
                GetMotionPlan,
                GetPlanningScene,
                GetPositionIK,
                GetStateValidity,
            )
            from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
        except ImportError as error:
            raise MoveGroupUnavailableError(
                "rclpy/moveit_msgs 不可用；请在装有 ROS 2 + MoveIt 的环境调用 plan()"
            ) from error
        self._session.messages = {
            "rclpy": rclpy,
            "Point": Point,
            "Pose": Pose,
            "PoseStamped": PoseStamped,
            "AttachedCollisionObject": AttachedCollisionObject,
            "CollisionObject": NativeObject,
            "Constraints": Constraints,
            "JointConstraint": JointConstraint,
            "OrientationConstraint": OrientationConstraint,
            "PlanningScene": PlanningScene,
            "PositionConstraint": PositionConstraint,
            "RobotState": RobotState,
            "ApplyPlanningScene": ApplyPlanningScene,
            "GetPlanningScene": GetPlanningScene,
            "PlanningSceneComponents": PlanningSceneComponents,
            "AllowedCollisionEntry": AllowedCollisionEntry,
            "GetMotionPlan": GetMotionPlan,
            "GetPositionIK": GetPositionIK,
            "GetStateValidity": GetStateValidity,
            "BoundingVolume": BoundingVolume,
            "Mesh": Mesh,
            "MeshTriangle": MeshTriangle,
            "SolidPrimitive": SolidPrimitive,
        }

    def _ensure_session(self) -> None:
        if self._ready:
            return
        self._import_native()
        messages = self._session.messages
        rclpy = messages["rclpy"]
        import os

        if self.config.domain_id is not None:
            current = os.environ.get("ROS_DOMAIN_ID")
            if current is not None and int(current) != int(self.config.domain_id):
                raise MoveGroupUnavailableError(
                    f"ROS_DOMAIN_ID={current} 与配置的 {self.config.domain_id} 不一致")
        if not rclpy.ok():
            rclpy.init(args=["--ros-args", "-r", f"__node:={self.config.node_name}"])
        node = rclpy.create_node(self.config.node_name)
        self._session.rclpy = rclpy
        self._session.node = node
        self._session.clients = {
            "plan": node.create_client(messages["GetMotionPlan"], _PLAN_SERVICE),
            "scene": node.create_client(messages["ApplyPlanningScene"], _SCENE_SERVICE),
            "get_scene": node.create_client(messages["GetPlanningScene"], "/get_planning_scene"),
            "validity": node.create_client(messages["GetStateValidity"], _VALIDITY_SERVICE),
            "ik": node.create_client(messages["GetPositionIK"], _IK_SERVICE),
        }
        self._ready = True

    @timed_stage("service_discovery")
    def wait_ready(self, timeout_s: float | None = None) -> bool:
        """等待四个 move_group 服务上线；不可用时抛 ``MoveGroupUnavailableError``。"""

        self._ensure_session()
        budget = float(timeout_s if timeout_s is not None else self.config.service_wait_s)
        deadline = time.monotonic() + budget
        for name, client in self._session.clients.items():
            remaining = max(0.5, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                raise MoveGroupUnavailableError(f"move_group 服务不可用: {name}")
        return True

    @timed_stage("rpc")
    def _call(self, name: str, request: Any, timeout_s: float | None = None) -> Any:
        client = self._session.clients[name]
        future = client.call_async(request)
        self._session.rclpy.spin_until_future_complete(
            self._session.node, future,
            timeout_sec=float(timeout_s if timeout_s is not None else self.config.service_timeout_s))
        if not future.done():
            raise TimeoutError(f"move_group 服务 {name} 调用超时")
        error = future.exception()
        if error is not None:
            raise RuntimeError(f"move_group 服务 {name} 调用失败: {error}")
        return future.result()

    # ------------------------------------------------------------------ 场景
    def _world_digest(self, world: CollisionWorld) -> str:
        """按对象身份缓存碰撞世界摘要（几十万三角面时不可每条请求重算）。"""

        # Retain the object: caching only id(world) can alias a collected snapshot.
        cached = self._world_digests.get(id(world))
        if cached is not None and cached[0] is world:
            return cached[1]
        digest = world.digest
        self._world_digests = {id(world): (world, digest)}
        return digest

    @timed_stage("scene_identity")
    def _scene_key(self, request: PlanRequest) -> tuple[Any, ...]:
        return (
            self._world_digest(request.collision_world),
            tuple((item.body_id, item.link_name, item.flange_from_body, item.touch_links)
                  for item in request.attachments),
            request.constraints.allowed_collision_pairs,
            request.collision_world.frame_id,
        )

    @timed_stage("scene_readback")
    def _read_scene(self, *, state_only: bool = False):
        messages = self._session.messages
        query = messages["GetPlanningScene"].Request()
        flags = messages["PlanningSceneComponents"]
        query.components.components = (flags.ROBOT_STATE | flags.SCENE_SETTINGS) if state_only else (
            flags.SCENE_SETTINGS |
            flags.WORLD_OBJECT_NAMES | flags.ROBOT_STATE | flags.ROBOT_STATE_ATTACHED_OBJECTS |
            flags.ALLOWED_COLLISION_MATRIX)
        return self._call("get_scene", query).scene

    @timed_stage("scene_sync")
    def _load_scene(self, request: PlanRequest) -> dict[str, Any]:
        """Apply an owned scene diff; commit caches only after native readback."""
        from .planning_scene_sync import apply_scene
        return apply_scene(self, request)

    def validate(self, scene: CollisionWorld) -> Mapping[str, Any]:
        """值对象级校验；不做碰撞查询，也不声称场景已被装载。"""

        if not isinstance(scene, CollisionWorld):
            raise TypeError("scene 必须是 CollisionWorld")
        issues: list[dict[str, Any]] = []
        mesh_triangles = 0
        for obj in scene.objects:
            for geometry in obj.geometry:
                if geometry.kind == "mesh":
                    if geometry.vertices is None or geometry.triangles is None:
                        issues.append({"object_id": obj.object_id, "kind": "file_mesh_unsupported",
                                       "mesh_path": geometry.mesh_path})
                        continue
                    mesh_triangles += len(geometry.triangles)
                elif geometry.kind not in ("box", "cylinder"):
                    issues.append({"object_id": obj.object_id, "kind": "unsupported_geometry",
                                   "geometry": geometry.kind})
        return {
            "status": "valid" if not issues else "invalid",
            "frame_id": scene.frame_id,
            "object_count": len(scene.objects),
            "mesh_triangles": mesh_triangles,
            "issues": issues,
            "collision_query": "not_run",
            "continuous_proof": False,
        }

    # ------------------------------------------------------------------ 请求
    def _state(self, request: PlanRequest, q: Sequence[float], rail: float | None = None) -> Any:
        """构造完整 ``RobotState``：六轴关节 + 声明时的导轨坐标。

        只发规划组关节会让 MoveIt 把导轨当成默认 0，这正是历史 harness
        的缺口；这里导轨坐标始终显式写入，未声明导轨轴时才没有该关节。
        """

        names = list(request.robot.joint_names)
        positions = [float(value) for value in q]
        if request.rail_axis is not None:
            value = request.start_rail_m if rail is None else rail
            if value is None:
                raise ValueError("请求声明了导轨轴却没有导轨坐标")
            names.append(request.rail_axis.axis_id)
            positions.append(float(value))
        state = self._session.messages["RobotState"]()
        state.joint_state.name = names
        state.joint_state.position = positions
        return state

    def _start_state(self, request: PlanRequest) -> Any:
        return self._state(request, request.start_q, request.start_rail_m)

    @timed_stage("attachment_message")
    def _attach_state_objects(self, request: PlanRequest, state: Any) -> Any:
        """把请求声明的附着体写进 ``RobotState``（状态校验必须看到载荷）。

        ``GetStateValidity`` 用的是请求里的 ``RobotState``；只发关节值会让
        move_group 用一个**不含附着体**的状态去做碰撞查询，于是"带着杯子"
        的零位移/逐路点复核看不到杯子。这里按 ``request.attachments`` 把同一
        个对象以 ``AttachedCollisionObject`` 形态写进状态，和
        ``_load_scene`` 装进 PlanningScene 的对象逐字段一致。
        """

        if not request.attachments:
            return state
        messages = self._session.messages
        by_id = {item.object_id: item for item in request.collision_world.objects}
        attached = []
        for spec in request.attachments:
            item = messages["AttachedCollisionObject"]()
            native = _object_to_native(by_id[spec.body_id])
            native.header.frame_id = spec.link_name
            native.pose = _matrix_to_native(spec.flange_from_body)
            item.link_name = spec.link_name
            item.object = native
            item.touch_links = list(spec.touch_links)
            attached.append(item)
        state.attached_collision_objects = attached
        return state

    def _state_with_scene_attachments(self, request, q, rail):
        """已核验的驻留场景保留原生形状；其余请求显式携带全部附着体。"""
        state = self._state(request, q, rail)
        if self._loaded_scene_key is not None and self._loaded_scene_key == self._scene_key(request):
            state.is_diff = True
            return state
        return self._attach_state_objects(request, state)

    def _group(self, request: PlanRequest) -> str:
        return request.planning_group or request.robot.planning_group

    def _goal_constraints(
        self,
        request: PlanRequest,
        *,
        joint_goal: Mapping[str, float] | None = None,
        pose: CartesianGoal | None = None,
    ) -> Any:
        messages = self._session.messages
        constraints = messages["Constraints"]()
        goal = request.goal
        if goal.kind == "joint" or joint_goal is not None:
            target = (
                joint_goal
                if joint_goal is not None
                else dict(zip(request.robot.joint_names, goal.joint_positions or (), strict=True))
            )
            tolerance = float(request.options.get("joint_goal_tolerance_rad", 1e-4))
            for name, value in target.items():
                item = messages["JointConstraint"]()
                item.joint_name = name
                item.position = float(value)
                item.tolerance_above = tolerance
                item.tolerance_below = tolerance
                item.weight = 1.0
                constraints.joint_constraints.append(item)
            return constraints
        assert pose is not None
        position = messages["PositionConstraint"]()
        position.header.frame_id = pose.frame_ref
        position.link_name = pose.link_name or request.robot.flange_link
        volume = messages["BoundingVolume"]()
        sphere = messages["SolidPrimitive"]()
        sphere.type = messages["SolidPrimitive"].SPHERE
        sphere.dimensions = [max(float(pose.position_tolerance_m), 1e-4)]
        volume.primitives.append(sphere)
        volume.primitive_poses.append(_pose_at(pose))
        position.constraint_region = volume
        position.weight = 1.0
        constraints.position_constraints.append(position)
        orientation = messages["OrientationConstraint"]()
        orientation.header.frame_id = pose.frame_ref
        orientation.link_name = position.link_name
        orientation.orientation.x = pose.orientation_xyzw[0]
        orientation.orientation.y = pose.orientation_xyzw[1]
        orientation.orientation.z = pose.orientation_xyzw[2]
        orientation.orientation.w = pose.orientation_xyzw[3]
        orientation.absolute_x_axis_tolerance = float(pose.orientation_tolerance_rad)
        orientation.absolute_y_axis_tolerance = float(pose.orientation_tolerance_rad)
        orientation.absolute_z_axis_tolerance = float(pose.orientation_tolerance_rad)
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _path_constraints(self, request: PlanRequest) -> Any:
        """Path constraints use rotation vectors in the explicitly named frame."""
        messages = self._session.messages
        constraints = messages["Constraints"]()
        for spec in request.constraints.orientation:
            item = messages["OrientationConstraint"]()
            item.header.frame_id = spec.frame_ref
            item.link_name = spec.link_name
            item.orientation.x, item.orientation.y, item.orientation.z, item.orientation.w = spec.orientation_xyzw
            tolerances=(spec.absolute_x_axis_tolerance_rad,spec.absolute_y_axis_tolerance_rad,
                        spec.absolute_z_axis_tolerance_rad)
            if self.config.native_orientation_error_frame == "desired_link":
                from .orientation_validation import native_local_tolerances
                tolerances=native_local_tolerances(spec.orientation_xyzw,tolerances)
            item.absolute_x_axis_tolerance,item.absolute_y_axis_tolerance,item.absolute_z_axis_tolerance=tolerances
            item.parameterization = item.ROTATION_VECTOR
            item.weight = spec.weight
            constraints.orientation_constraints.append(item)
        return constraints

    @timed_stage("ik")
    def _compute_ik(self, request: PlanRequest) -> dict[str, Any]:
        """位姿目标的逆解探测：只用来解释「为什么规划失败」，不单独否决请求。

        位姿目标可能有多个 IK 分支；单次采样落在碰撞分支上并不代表目标不可达
        （实测 ``home`` 姿态在同一请求里既有碰撞解也有可行解）。因此这里用
        起始状态与若干固定种子各求一次解，把结果作为证据记录，**是否成功仍由
        ``/plan_kinematic_path`` 决定**；只有当所有采样都无解时才判 ``unreachable``。
        """

        messages = self._session.messages
        goal = request.goal
        if goal.kind != "pose" or goal.pose is None:
            return {"checked": False, "reason": "joint_goal_has_no_ik_step"}
        pose = goal.pose
        seeds: list[tuple[float, ...]] = [tuple(request.start_q)]
        for candidate in request.seeds:
            if candidate not in seeds:
                seeds.append(tuple(candidate))
        neutral = tuple(0.0 for _ in request.robot.joint_names)
        if neutral not in seeds:
            seeds.append(neutral)
        seeds = seeds[:3]
        samples: list[dict[str, Any]] = []
        for seed in seeds:
            record: dict[str, Any] = {"seed_q": [round(float(value), 6) for value in seed]}
            ik_request = messages["GetPositionIK"].Request()
            ik_request.ik_request.group_name = self._group(request)
            ik_request.ik_request.robot_state = self._state(
                request, seed, request.goal_rail_m
            )
            ik_request.ik_request.avoid_collisions = False
            ik_request.ik_request.ik_link_name = pose.link_name or request.robot.flange_link
            stamped = messages["PoseStamped"]()
            stamped.header.frame_id = pose.frame_ref
            stamped.pose = _pose_at(pose)
            ik_request.ik_request.pose_stamped = stamped
            ik_request.ik_request.timeout = _duration(self.config.ik_timeout_s)
            response = self._call("ik", ik_request)
            code = int(response.error_code.val)
            record["error_code"] = code
            # ``MOVEIT_ERROR_CODES`` 的值是 ``MoveItErrorCode`` 值对象（不是
            # ``(status, category)`` 元组）；这里曾按下标取第二项，任何 IK 失败
            # 都会抛 TypeError 并让整条相位变成 planner_exception。
            record["failure_category"] = describe_moveit_error(code).category
            record["avoid_collisions"] = False
            record["ik_link"] = ik_request.ik_request.ik_link_name
            if code == 1:
                names = list(response.solution.joint_state.name)
                positions = [float(value) for value in response.solution.joint_state.position]
                by_name = dict(zip(names, positions, strict=True))
                missing = [name for name in request.robot.joint_names if name not in by_name]
                if missing:
                    record["failure_category"] = "ik_solution_missing_joint"
                    record["missing_joints"] = missing
                else:
                    record["solution_q"] = [by_name[name] for name in request.robot.joint_names]
                    record["solution_validity"] = self._state_validity(
                        request, record["solution_q"], request.goal_rail_m)
            samples.append(record)
        solved = [item for item in samples if "solution_q" in item]
        valid = [item for item in solved if item.get("solution_validity", {}).get("valid")]
        return {
            "checked": True,
            "samples": samples,
            "any_solution": bool(solved),
            "any_valid_solution": bool(valid),
            "first_valid_q": valid[0]["solution_q"] if valid else None,
            "colliding_solutions": sum(
                1 for item in solved if not item.get("solution_validity", {}).get("valid")),
            "seeds_probed": len(samples),
        }

    @timed_stage("state_validation")
    def _state_validity(
        self,
        request: PlanRequest,
        q: Sequence[float],
        rail: float | None = None,
        *,
        whole_robot: bool = False,
    ) -> dict[str, Any]:
        messages = self._session.messages
        validity = messages["GetStateValidity"].Request()
        validity.group_name = "" if whole_robot else self._group(request)
        validity.constraints = self._path_constraints(request)
        validity.robot_state = self._state_with_scene_attachments(request, q, rail)
        response = self._call("validity", validity)
        orientation_checks=[]
        if request.constraints.orientation:
            from .orientation_validation import check_orientation_constraints
            external={request.rail_axis.axis_id:float(request.start_rail_m if rail is None else rail)} if request.rail_axis else {}
            orientation_checks=check_orientation_constraints(request.robot,q,external,request.constraints.orientation)
        return {
            "valid": bool(response.valid) and all(row["satisfied"] for row in orientation_checks),
            "orientation_constraints":orientation_checks,
            "native_constraint_results":[{"satisfied":item.result,"distance":item.distance}
                for item in getattr(response,"constraint_result",())],
            "contacts": [f"{item.contact_body_1}/{item.contact_body_2}"
                         for item in response.contacts][:12],
            "contact_details": [
                {"body_1":item.contact_body_1,"body_2":item.contact_body_2,
                 "depth_m":float(item.depth),
                 "position_m":[item.position.x,item.position.y,item.position.z]}
                for item in response.contacts],
        }

    @timed_stage("dense_validation")
    def _dense_check(
        self,
        request: PlanRequest,
        points: Sequence[Mapping[str, Any]],
    ) -> tuple[int, dict[str, Any] | None]:
        """逐段插值导轨与六轴，按**整机**状态查世界碰撞。"""

        checks = 0
        step = self.config.max_joint_sample_step_rad
        rail_step = float(request.options.get("rail_sample_step_m", 0.01))
        for left, right in zip(points, points[1:]):
            q_left, q_right = left["q"], right["q"]
            rail_left, rail_right = float(left["rail"]), float(right["rail"])
            intervals = max(
                1,
                math.ceil(
                    max(abs(a - b) for a, b in zip(q_left, q_right, strict=True)) / step
                ),
                math.ceil(abs(rail_right - rail_left) / rail_step),
            )
            for index in range(1, intervals + 1):
                q = [
                    a + (b - a) * index / intervals
                    for a, b in zip(q_left, q_right, strict=True)
                ]
                rail = rail_left + (rail_right - rail_left) * index / intervals
                checks += 1
                hit = self._state_validity(request, q, rail, whole_robot=True)
                if not hit["valid"]:
                    return checks, {"failure_category": "dense_path_collision",
                                    "failed_q": q, "failed_rail_m": rail,
                                    "collision": hit}
        if points:
            checks += 1
            hit = self._state_validity(
                request, points[-1]["q"], points[-1]["rail"], whole_robot=True
            )
            if not hit["valid"]:
                return checks, {"failure_category": "goal_collision",
                                "failed_q": list(points[-1]["q"]),
                                "failed_rail_m": float(points[-1]["rail"]),
                                "collision": hit}
        return checks, None

    # ------------------------------------------------------------------ 规划
    def plan(self, request: PlanRequest) -> PlanResult:
        """规划一个请求；不执行、不返回空成功轨迹，并回显携带的载荷身份。

        载荷回显（``PlanResult.payload_ref``/``payload_profile_digest``/
        ``attachment_digest``）由 ``PlanResult.with_payload`` 统一附加：本后端
        在 ``_load_scene`` 里把请求声明的附着对象（含载荷）装进 PlanningScene，
        所以它有资格回显；没有声明载荷的请求结果不带这些字段。
        """

        if not isinstance(request, PlanRequest):
            raise TypeError("request 必须是 PlanRequest")
        before = self._timing.snapshot()
        started = time.perf_counter()
        result = self._plan_body(request)
        diagnostics = dict(result.diagnostics, performance={
            "schema": "unilab.planning-timing/v1",
            "wall_s": time.perf_counter() - started,
            "stages": self._timing.since(before),
            "stage_times_are_inclusive": True,
            "rpc_includes_transport_and_server": True,
            "pure_fcl_s": None,
        })
        return replace(result, diagnostics=diagnostics, digest="").with_payload(request)

    def _plan_body(self, request: PlanRequest) -> PlanResult:
        """规划主体：场景装载、IK、分段规划与逐路点碰撞复核。"""

        primitive = request.options.get("primitive")
        pipeline = request.options.get("pipeline", self.config.pipeline)
        planner_id = request.options.get("planner_id", self.config.planner_id)
        if primitive not in (None, "joint_ptp", "cartesian_linear", "rail"):
            return PlanResult(status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category":"unsupported_motion_primitive","primitive":primitive})
        if primitive == "cartesian_linear" and (pipeline != "pilz" or planner_id != "LIN" or request.rail_moves):
            return PlanResult(status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category":"cartesian_linear_requires_fixed_rail_Pilz_LIN"})
        if request.constraints.clearance is not None:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "clearance_constraint_not_supported",
                    "reason": "move_group adapter does not natively enforce the requested "
                              "clearance; refusing rather than silently ignoring it",
                },
            )
        started = time.monotonic()
        try:
            self._ensure_session()
            self.wait_ready()
            scene = self._load_scene(request)
            ik = self._compute_ik(request)
        except MoveGroupUnavailableError as error:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "planner_unavailable", "error": str(error),
                             "error_type": type(error.__cause__).__name__
                             if error.__cause__ is not None else None})
        except ImportError as error:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "planner_dependency_missing",
                             "error": str(error), "error_type": type(error).__name__})
        except ValueError as error:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "invalid_planner_configuration",
                             "error": str(error), "error_type": type(error).__name__})
        except (TimeoutError, RuntimeError) as error:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "move_group_service_failure",
                             "error": str(error), "error_type": type(error).__name__})
        if ik.get("checked") and not ik["any_solution"]:
            return PlanResult(
                status="unreachable", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "ik_no_solution",
                             "ik": ik, "scene": scene, "planned": False})

        zero_motion = _zero_motion(request)
        if zero_motion:
            # 零位移请求（例如 S09 门架动作期间机械臂保持不动）不交给规划器：
            # OMPL/TOTG 会把单点轨迹判成 invalid。这里只做状态有效性校验，并在
            # diagnostics 里写清楚这不是规划出来的轨迹。
            validity = self._state_validity(
                request, request.start_q, request.start_rail_m, whole_robot=True
            )
            status: PlanStatus = "success" if validity["valid"] else "collision"
            rail = (
                float(request.start_rail_m)
                if request.start_rail_m is not None
                else 0.0
            )
            waypoints = (PlanWaypoint(tuple(request.start_q), rail,
                                      0.0, "seed"),) if validity["valid"] else ()
            return PlanResult(
                status=status, waypoints=waypoints, planner=self.identity,
                collision_checks=1,
                diagnostics={
                    "failure_category": None if validity["valid"] else "hold_state_in_collision",
                    "collision": validity,
                    "ik": ik,
                    "scene": scene,
                    "zero_motion": True,
                    "collision_sampling": "single_state_validation_not_a_plan",
                    "planned": False,
                    "executed": False,
                })

        phases = self._phases(request)
        if phases is None:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={
                    "failure_category": "rail_planning_group_missing",
                    "reason": "request moves the rail but the SRDF has no group whose "
                              "joint set is exactly {rail_joint}; refusing instead of "
                              "planning the rail inside the six-axis group",
                    "rail_joint": request.rail_axis.axis_id if request.rail_axis else None,
                    "ik": ik, "scene": scene, "planned": False})

        collected: list[dict[str, Any]] = []
        segments: list[dict[str, Any]] = []
        offset = 0.0
        for phase in phases:
            segment = self._native_segment(request, **phase)
            summary = {
                key: segment.get(key)
                for key in ("group", "status", "error_code", "error_name",
                            "failure_category", "planning_time_s", "start_error", "times")
            }
            summary["points"] = len(segment.get("points", ()))
            segments.append(summary)
            if not segment["ok"]:
                return PlanResult(
                    status=segment["status"],
                    waypoints=tuple(self._waypoints(collected)),
                    planner=self.identity,
                    collision_checks=0,
                    diagnostics={
                        "failure_category": segment["failure_category"],
                        "moveit_error_code": segment.get("error_code"),
                        "moveit_error_name": segment.get("error_name"),
                        "moveit_error_message": segment.get("error_message"),
                        "rail_m": [float(request.start_rail_m or 0.0),
                                   float(request.goal_rail_m or 0.0)],
                        "segments": segments,
                        "ik": ik, "scene": scene,
                        "planned": False})
            if offset > 0.0:
                for point, timestamp in zip(
                        segment["points"], segment["times"], strict=True):
                    point["time"] = offset + timestamp
                segment["times"] = [offset + value for value in segment["times"]]
            offset = (
                segment["points"][-1]["time"] if segment["points"] else offset
            )
            next_points = segment["points"]
            if collected and next_points:
                left, right = collected[-1], next_points[0]
                if max(abs(a-b) for a,b in zip(left["q"],right["q"])) > 1e-5 or abs(left["rail"]-right["rail"])>1e-6:
                    return PlanResult(status="invalid",waypoints=(),planner=self.identity,
                        diagnostics={"failure_category":"segment_boundary_discontinuity"})
                next_points = next_points[1:]
            collected.extend(next_points)
        if not collected:
            return PlanResult(
                status="invalid", waypoints=(), planner=self.identity,
                diagnostics={"failure_category": "empty_or_malformed_trajectory",
                             "segments": segments, "ik": ik, "scene": scene,
                             "planned": False})
        waypoints = tuple(self._waypoints(collected))
        checks, failure = self._dense_check(request, collected)
        if failure is not None:
            return PlanResult(
                status="collision", waypoints=waypoints, planner=self.identity,
                collision_checks=checks,
                diagnostics={**failure, "ik": ik, "scene": scene,
                             "segments": segments,
                             "planning_wall_time_s": time.monotonic() - started,
                             "planned": True})
        return PlanResult(
            status="success", waypoints=waypoints, planner=self.identity,
            collision_checks=checks,
            diagnostics={
                "group": self._group(request),
                "joint_names": list(request.robot.joint_names),
                "rail_axis": request.rail_axis.as_mapping() if request.rail_axis else None,
                "start_rail_m": request.start_rail_m,
                "goal_rail_m": request.goal_rail_m,
                "phases": segments,
                "ik": ik,
                "scene": scene,
                "planning_wall_time_s": time.monotonic() - started,
                "waypoint_count": len(waypoints),
                "collision_sampling": "whole_robot_dense_joint_and_rail_not_continuous_proof",
                "executed": False,
                "hardware": False,
            },
        )

    def _phases(self, request: PlanRequest) -> list[dict[str, Any]] | None:
        """把一条请求拆成「导轨段 + 六轴段」；导轨段只含导轨关节。

        合同要求导轨是独立轴，因此导轨移动必须由只含该关节的规划组完成，
        六轴段则把导轨锁在段末坐标。返回 ``None`` 表示请求要动导轨但 SRDF
        没有独立导轨组（拒绝，而不是退回导轨 0）。
        """

        start_q = [float(value) for value in request.start_q]
        start_rail = float(request.start_rail_m or 0.0)
        goal_rail = float(request.goal_rail_m or 0.0)
        rail_joint = request.rail_axis.axis_id if request.rail_axis else None
        phases: list[dict[str, Any]] = []
        if request.rail_moves and rail_joint is not None:
            rail_group = request.options.get("rail_planning_group") or rail_planning_group(
                request.robot, rail_joint
            )
            if not rail_group:
                return None
            phases.append({
                "group": str(rail_group),
                "start_q": start_q,
                "joint_goal": {rail_joint: goal_rail},
                "pose": None,
                "rail_start": start_rail,
                "rail_goal": goal_rail,
            })
        arm_start = start_q
        arm_rail = goal_rail if request.rail_moves else start_rail
        goal = request.goal
        if goal.kind == "joint":
            assert goal.joint_positions is not None
            joint_goal = dict(
                zip(request.robot.joint_names, goal.joint_positions, strict=True)
            )
            if max(abs(a - b) for a, b in zip(
                    goal.joint_positions, arm_start, strict=True)) <= 1e-9:
                return phases
            phases.append({
                "group": self._group(request),
                "start_q": arm_start,
                "joint_goal": joint_goal,
                "pose": None,
                "rail_start": arm_rail,
                "rail_goal": arm_rail,
            })
            return phases
        phases.append({
            "group": self._group(request),
            "start_q": arm_start,
            "joint_goal": None,
            "pose": goal.pose,
            "rail_start": arm_rail,
            "rail_goal": arm_rail,
        })
        return phases

    @staticmethod
    def _waypoints(points: Sequence[Mapping[str, Any]]) -> list[PlanWaypoint]:
        return [
            PlanWaypoint(tuple(point["q"]), float(point["rail"]), float(point["time"]),
                         "planner", velocities=point.get("velocities"),
                         accelerations=point.get("accelerations"),
                         rail_velocity=point.get("rail_velocity"),
                         rail_acceleration=point.get("rail_acceleration"))
            for point in points
        ]

    @timed_stage("native_segment")
    def _native_segment(
        self,
        request: PlanRequest,
        *,
        group: str,
        start_q: Sequence[float],
        joint_goal: Mapping[str, float] | None,
        pose: CartesianGoal | None,
        rail_start: float,
        rail_goal: float,
    ) -> dict[str, Any]:
        from .planning_scene_sync import sync_robot_state
        sync_robot_state(self, request, q=start_q, rail=rail_start)
        messages = self._session.messages
        options = dict(request.options)
        pipeline = str(options.get("pipeline", self.config.pipeline))
        planner_id = str(options.get("planner_id", self.config.planner_id))
        attempts = int(options.get("planning_attempts", self.config.planning_attempts))
        planning_time = float(options.get(
            "planning_time_s", min(float(request.timeout_s), self.config.planning_time_s)))
        motion = messages["GetMotionPlan"].Request().motion_plan_request
        motion.group_name = group
        motion.pipeline_id = pipeline
        motion.planner_id = planner_id
        motion.num_planning_attempts = attempts
        motion.allowed_planning_time = planning_time
        velocity_scale = float(options.get("velocity_scale", self.config.velocity_scaling))
        acceleration_scale = float(options.get("acceleration_scale", self.config.acceleration_scaling))
        if not (0.0 < velocity_scale <= 1.0 and 0.0 < acceleration_scale <= 1.0):
            return {"ok":False,"status":"invalid","group":group,
                    "failure_category":"invalid_motion_scaling"}
        motion.max_velocity_scaling_factor = velocity_scale
        motion.max_acceleration_scaling_factor = acceleration_scale
        motion.start_state = self._state_with_scene_attachments(request, start_q, rail_start)
        motion.goal_constraints.append(
            self._goal_constraints(request, joint_goal=joint_goal, pose=pose)
        )
        motion.path_constraints = self._path_constraints(request)
        request_message = messages["GetMotionPlan"].Request()
        request_message.motion_plan_request = motion
        started = time.monotonic()
        first_attempt: dict[str, Any] | None = None
        for attempt in range(2):
            try:
                response = self._call(
                    "plan", request_message,
                    timeout_s=planning_time * attempts + self.config.service_timeout_s)
            except Exception as error:  # noqa: BLE001 - 服务异常需要原始类型作为证据
                return {"ok": False, "status": "invalid", "group": group,
                        "failure_category": "move_group_service_failure",
                        "error": f"{type(error).__name__}: {error}",
                        "planning_time_s": time.monotonic() - started}
            result = response.motion_plan_response
            code = int(result.error_code.val)
            if code == 1 or code not in _TRANSIENT_ERROR_CODES or attempt == 1:
                break
            # START_STATE_INVALID/CRASH/PREEMPTED 是 move_group 侧的瞬时响应（同一
            # 请求重发即可）；重试一次并保留第一次的原始码。
            failure = describe_moveit_error(code)
            first_attempt = {"error_code": code, "error_name": failure.name,
                             "failure_category": failure.category}
            result = response.motion_plan_response
        elapsed = time.monotonic() - started
        if code != 1:
            failure = describe_moveit_error(code)
            return {"ok": False, "status": failure.status, "group": group,
                    "failure_category": failure.category,
                    "error_code": code, "error_name": failure.name,
                    "error_message": failure.message,
                    "first_attempt": first_attempt,
                    "planning_time_s": float(result.planning_time or elapsed)}
        trajectory = result.trajectory.joint_trajectory
        names = list(trajectory.joint_names)
        known = set(request.robot.joint_names)
        if request.rail_axis is not None:
            known.add(request.rail_axis.axis_id)
        unknown = [name for name in names if name not in known]
        expected = {request.rail_axis.axis_id} if request.rail_axis and group == rail_planning_group(request.robot, request.rail_axis.axis_id) else set(request.robot.joint_names)
        if not names or unknown or len(names) != len(set(names)) or set(names) != expected:
            return {"ok": False, "status": "invalid", "group": group,
                    "failure_category": "trajectory_joint_names_mismatch",
                    "actual_joint_names": names, "expected_joint_names": sorted(known)}
        rail_joint = request.rail_axis.axis_id if request.rail_axis else None
        points: list[dict[str, Any]] = []
        times: list[float] = []
        for point in trajectory.points:
            mapping = dict(zip(names, [float(value) for value in point.positions],
                               strict=True))
            q = [
                mapping[name] if name in mapping else float(start_q[index])
                for index, name in enumerate(request.robot.joint_names)
            ]
            if rail_joint is not None:
                rail = mapping.get(rail_joint, float(rail_start))
            else:
                rail = 0.0
            row = {"q": q, "rail": rail}
            for native_name, rail_name in (("velocities", "rail_velocity"),
                                           ("accelerations", "rail_acceleration")):
                values = list(getattr(point, native_name, ()))
                if values:
                    if len(values) != len(names):
                        return {"ok": False, "status": "invalid", "group": group,
                                "failure_category": "malformed_trajectory_derivatives"}
                    derivative = dict(zip(names, values, strict=True))
                    row[native_name] = tuple(float(derivative.get(name, 0.0))
                                             for name in request.robot.joint_names)
                    row[rail_name] = float(derivative.get(rail_joint, 0.0))
            points.append(row)
            times.append(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)
        if not points:
            return {"ok": False, "status": "invalid", "group": group,
                    "failure_category": "empty_or_malformed_trajectory",
                    "error_code": code}
        if (len(times) < 2 or abs(times[0]) > 1e-9
                or any(not math.isfinite(t) for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            return {"ok": False, "status": "invalid", "group": group,
                    "failure_category": "invalid_trajectory_timing", "times": times}
        if max(abs(a-b) for a,b in zip(points[0]["q"],start_q)) > 1e-5 or abs(points[0]["rail"]-rail_start)>1e-6:
            return {"ok": False, "status": "invalid", "group": group,
                    "failure_category": "trajectory_start_discontinuity",
                    "start_error": {"requested_q":list(start_q),"native_q":points[0]["q"],
                                    "requested_rail":rail_start,"native_rail":points[0]["rail"]}}
        for point, timestamp in zip(points, times, strict=True):
            point["time"] = float(timestamp)
        return {"ok": True, "points": points, "times": times, "group": group,
                "error_code": code,
                "planning_time_s": float(result.planning_time or elapsed),
                "pipeline": pipeline, "planner_id": planner_id}

    # ------------------------------------------------------------------ 收尾
    def close(self) -> None:
        """释放节点与 rclpy 会话；可重复调用。"""

        node = self._session.node
        self._session.node = None
        self._ready = False
        self._loaded_scene_key = None
        self._scene_receipt = None
        self._loaded_ids.clear()
        self._attached_ids.clear()
        self._allowed_pairs.clear()
        self._loaded_objects.clear()
        self._loaded_attachment_specs.clear()
        self._world_digests.clear()
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # pragma: no cover - 关闭阶段的异常不影响结论
                pass

    def __enter__(self) -> MoveGroupPlanner:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


def _zero_motion(request: PlanRequest) -> bool:
    """目标已在显式关节容差内时保留实际状态，不制造零时长运动。"""

    if request.rail_moves:
        return False
    goal = request.goal
    if goal.kind != "joint" or goal.joint_positions is None:
        return False
    return max(abs(left - right)
               for left, right in zip(goal.joint_positions, request.start_q, strict=True)) <= float(request.options.get("joint_goal_tolerance_rad", 1e-4))


def _pose_at(pose: CartesianGoal) -> Any:
    from geometry_msgs.msg import Pose

    item = Pose()
    item.position.x, item.position.y, item.position.z = pose.xyz_m
    item.orientation.x, item.orientation.y = pose.orientation_xyzw[0], pose.orientation_xyzw[1]
    item.orientation.z, item.orientation.w = pose.orientation_xyzw[2], pose.orientation_xyzw[3]
    return item


def _duration(seconds: float) -> Any:
    from builtin_interfaces.msg import Duration

    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


__all__ = [
    "MoveGroupPlanner",
    "MoveGroupPlannerConfig",
    "MoveGroupUnavailableError",
    "build_move_group_parameters",
    "detect_planner_identity",
    "rail_planning_group",
]
