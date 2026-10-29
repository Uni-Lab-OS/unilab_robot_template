"""MoveItPy/FCL 规划器：只暴露 ``PlanRequest``/``PlanResult``。

该模块在导入时不依赖 ROS、MoveIt、FCL 或 NumPy。只有第一次真正调用
``plan`` 时才导入 MoveItPy；没有 ROS/MoveIt 的环境仍可导入合同、运行
纯校验和构造测试替身。真实规划失败的原始异常会保留在 ``__cause__``，
同时以 ``PlannerUnavailableError`` 或 ``PlanResult.diagnostics`` 暴露。

本实现只做规划，不创建控制器、不调用 ``execute``、不连接硬件。
"""

from __future__ import annotations

import copy
import hashlib
import importlib
import importlib.metadata
import math
import struct
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from unilab_robot_contracts.planning import (
    CartesianGoal,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    MotionPlanner,
    PlanRequest,
    PlanResult,
    PlanStatus,
    PlanWaypoint,
    PlannerIdentity,
    RobotModel,
    SceneValidatingPlanner,
)

from .moveit_error_codes import describe_moveit_error, status_for_moveit_error
from .moveit_ompl_config import OMPL_PLANNING_PLUGIN, require_ompl_planning_plugin

_PLANNER_NAME = "MoveItPy"
_IMPLEMENTATION = "moveit_py_fcl"
_DEFAULT_GROUP = "arm"


class PlannerUnavailableError(RuntimeError):
    """当前解释器没有可用的 MoveItPy。原始 ImportError 保留为 cause。"""


class MoveItPlanningError(RuntimeError):
    """MoveIt 原生规划抛出异常，或返回了无法解释的结果。"""


@dataclass(frozen=True, slots=True)
class MoveItPlannerConfig:
    """`MoveItPlanner` 的非场景调参。

    ``pipeline``/``planner_id`` 必须是目标 MoveIt 实例实际加载的名称；
    未知名称由 MoveIt 在规划时拒绝，不会静默回退。
    """

    node_name: str = "unilab_planning"
    pipeline: str = "ompl"
    planner_id: str = "RRTConnectkConfigDefault"
    planning_time_s: float = 10.0
    planning_attempts: int = 1
    velocity_scaling: float = 0.2
    acceleration_scaling: float = 0.2
    max_joint_sample_step_rad: float = 0.02
    collision_contacts: int = 30
    fallback_step_m: float = 1e-3

    def __post_init__(self) -> None:
        if not str(self.node_name).strip():
            raise ValueError("node_name 不能为空")
        if not str(self.pipeline).strip() or not str(self.planner_id).strip():
            raise ValueError("pipeline/planner_id 不能为空")
        for attr in ("planning_time_s", "velocity_scaling", "acceleration_scaling"):
            value = float(getattr(self, attr))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{attr} 必须是正有限数")
            object.__setattr__(self, attr, value)
        for attr in ("planning_attempts", "collision_contacts"):
            value = getattr(self, attr)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{attr} 必须是正整数")
        for attr in ("max_joint_sample_step_rad", "fallback_step_m"):
            value = float(getattr(self, attr))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{attr} 必须是正有限数")
            object.__setattr__(self, attr, value)


def _distribution_version(*names: str) -> str:
    for name in names:
        try:
            return importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
        except Exception:  # pragma: no cover - broken metadata is environment-specific
            continue
    return "unknown"


def _module_version(module_name: str, distribution_names: tuple[str, ...]) -> str:
    version = _distribution_version(*distribution_names)
    if version != "unknown":
        return version
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return "unknown"
    value = getattr(module, "__version__", None)
    return str(value) if value is not None else "unknown"


def detect_planner_identity() -> PlannerIdentity:
    """读取已安装引擎的版本，不导入 MoveIt 或初始化 ROS。"""

    moveit_version = _distribution_version(
        "moveit", "moveit-core", "moveit_py", "moveit-py"
    )
    return PlannerIdentity(
        name=_PLANNER_NAME,
        implementation=_IMPLEMENTATION,
        version=moveit_version,
        engine_versions={
            "moveit": moveit_version,
            "moveit_msgs": _distribution_version("moveit_msgs"),
            "fcl": _distribution_version("fcl", "python-fcl", "libfcl"),
            "ompl": _module_version("ompl", ("ompl", "ompl-py")),
        },
    )


def _pose_matrix_to_message(matrix: object) -> Any:
    """把合同 4x4 矩阵转换为 geometry_msgs/Pose；仅原生路径调用。"""

    from geometry_msgs.msg import Pose

    rows = tuple(tuple(float(value) for value in row) for row in matrix)  # type: ignore[union-attr]
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
        scale = math.sqrt(
            max(
                0.0,
                1.0
                + rotation[index][index]
                - rotation[next_index][next_index]
                - rotation[last_index][last_index],
            )
        ) * 2.0
        quaternion = [0.0, 0.0, 0.0, 0.0]
        quaternion[index] = scale / 4.0
        quaternion[next_index] = (
            rotation[next_index][index] + rotation[index][next_index]
        ) / scale
        quaternion[last_index] = (
            rotation[last_index][index] + rotation[index][last_index]
        ) / scale
        quaternion[3] = (
            rotation[last_index][next_index] - rotation[next_index][last_index]
        ) / scale
        qx, qy, qz, qw = quaternion
    pose = Pose()
    pose.position.x = rows[0][3]
    pose.position.y = rows[1][3]
    pose.position.z = rows[2][3]
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _mesh_from_memory(geometry: CollisionGeometry) -> Any:
    from geometry_msgs.msg import Point
    from shape_msgs.msg import Mesh, MeshTriangle

    if geometry.vertices is None or geometry.triangles is None:
        raise ValueError("内存网格缺少 vertices/triangles")
    scale = geometry.mesh_scale
    vertices = [
        Point(
            x=vertex[0] * scale[0],
            y=vertex[1] * scale[1],
            z=vertex[2] * scale[2],
        )
        for vertex in geometry.vertices
    ]
    triangles = [
        MeshTriangle(vertex_indices=[int(index) for index in triangle])
        for triangle in geometry.triangles
    ]
    mesh = Mesh()
    mesh.vertices = vertices
    mesh.triangles = triangles
    return mesh


def _read_stl(path: Path) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, int, int], ...]]:
    """读取二进制 STL；ASCII STL 交给显式错误，避免静默误解析。"""

    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"STL 文件过短: {path}")
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) < 84 + count * 50:
        raise ValueError(f"STL 二进制长度与三角面数不符: {path}")
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    offset = 84
    for _ in range(count):
        values = struct.unpack_from("<12f", raw, offset)
        offset += 50
        base = len(vertices)
        for index in range(3):
            vertices.append(
                (
                    float(values[3 + index * 3]),
                    float(values[4 + index * 3]),
                    float(values[5 + index * 3]),
                )
            )
        triangles.append((base, base + 1, base + 2))
    return tuple(vertices), tuple(triangles)


def _geometry_to_native(geometry: CollisionGeometry) -> tuple[Any, Any, Any]:
    """返回 (mesh|None, primitive|None, pose|None)。"""

    if geometry.kind == "box":
        from shape_msgs.msg import SolidPrimitive

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(geometry.size_m or ())
        return None, primitive, None
    if geometry.kind == "cylinder":
        from shape_msgs.msg import SolidPrimitive

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(geometry.length_m), float(geometry.radius_m)]
        return None, primitive, None
    if geometry.vertices is not None:
        return _mesh_from_memory(geometry), None, None
    assert geometry.mesh_path is not None
    path = Path(geometry.mesh_path)
    if not path.is_file():
        raise ValueError(f"网格文件不存在: {path}")
    if path.suffix.lower() != ".stl":
        raise ValueError(f"默认 MoveItPlanner 只直接读取 STL: {path.suffix}")
    vertices, triangles = _read_stl(path)
    mesh_geometry = CollisionGeometry(
        kind="mesh",
        vertices=vertices,
        triangles=triangles,
        mesh_scale=geometry.mesh_scale,
    )
    return _mesh_from_memory(mesh_geometry), None, None


def _object_to_native(obj: CollisionObject, *, frame_id: str = "world") -> Any:
    from moveit_msgs.msg import CollisionObject as NativeCollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle

    native = NativeCollisionObject()
    native.id = obj.object_id
    native.header.frame_id = frame_id
    native.pose = _pose_matrix_to_message(obj.world_pose)
    native.operation = NativeCollisionObject.ADD
    combined_mesh = Mesh()
    for geometry in obj.geometry:
        mesh, primitive, primitive_pose = _geometry_to_native(geometry)
        if mesh is not None:
            offset = len(combined_mesh.vertices)
            combined_mesh.vertices.extend(mesh.vertices)
            combined_mesh.triangles.extend(
                MeshTriangle(
                    vertex_indices=[index + offset for index in triangle.vertex_indices]
                )
                for triangle in mesh.triangles
            )
        elif primitive is not None:
            native.primitives.append(primitive)
            native.primitive_poses.append(
                _pose_matrix_to_message(primitive_pose or ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
            )
    if combined_mesh.triangles:
        native.meshes.append(combined_mesh)
        native.mesh_poses.append(_pose_matrix_to_message(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))))
    if not native.meshes and not native.primitives:
        raise ValueError(f"对象 {obj.object_id} 没有可装载几何")
    return native


class MoveItPlanner:
    """MoveItPy 后端；第一次 ``plan`` 才导入原生模块。

    参数 ``config`` 只控制规划管线/速度/采样，不改变请求中的碰撞世界、
    关节限位或安全姿态。一个实例可连续规划多个请求；同一机器人模型只在
    首次使用时初始化一次 MoveItPy。
    """

    def __init__(self, config: MoveItPlannerConfig | None = None) -> None:
        self.config = config if config is not None else MoveItPlannerConfig()
        self._robot: Any = None
        self._robot_model_digest: str | None = None
        self._component: Any = None
        self._monitor: Any = None
        self._loaded_world_digest: str | None = None
        self._loaded_scene_key: tuple[Any, ...] | None = None
        self._loaded_object_ids: set[str] = set()
        self._attached_ids: set[str] = set()
        self._allowed_pairs: set[tuple[str, str]] = set()
        self._native: tuple[Any, Any, Any] | None = None
        self._identity: PlannerIdentity | None = None
        self.planning_identity = detect_planner_identity()

    @property
    def identity(self) -> PlannerIdentity:
        """返回规划器身份（无需初始化 MoveIt）。"""

        return self._identity or self.planning_identity

    def _require_native(self) -> tuple[Any, Any, Any]:
        if self._native is not None:
            return self._native
        try:
            from moveit.planning import MoveItPy, PlanRequestParameters
            from moveit.core.robot_state import RobotState
        except ImportError as error:
            raise PlannerUnavailableError(
                "MoveItPy 不可用；请在已安装 ROS/MoveIt 的环境调用 plan()"
            ) from error
        self._native = (MoveItPy, PlanRequestParameters, RobotState)
        return self._native

    def _build_config(self, request: PlanRequest) -> dict[str, Any]:
        robot = request.robot
        group = request.planning_group or robot.planning_group
        if request.goal.kind == "pose" and not robot.kinematics:
            raise ValueError("位姿目标必须在 RobotModel.kinematics 中声明 IK 求解器")
        pipeline = self.config.pipeline
        # MoveIt ≤2.5.x 只读单数 `planning_plugin`（缺省时按 pluginlib 声明顺序
        # 取第一个，即 chomp_interface/CHOMPPlanner，静默退回 CHOMP）；2.10+ 用
        # 生成的参数监听器只读复数 `planning_plugins`，未知的单数键被忽略。
        # 两个都写才能同时覆盖两边，与 `build_move_group_parameters` 一致。
        ompl = {
            "planning_plugin": OMPL_PLANNING_PLUGIN,
            "planning_plugins": [OMPL_PLANNING_PLUGIN],
            "planner_configs": {
                self.config.planner_id: {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                }
            },
            group: {
                "default_planner_config": self.config.planner_id,
                "planner_configs": [self.config.planner_id],
                "longest_valid_segment_fraction": 0.005,
            },
        }
        # 缺少单数键或只给复数键时 MoveIt ≤2.5.x 会静默回落到 CHOMP；
        # 在发出配置前显式失败，而不是等运行日志里才发现。
        require_ompl_planning_plugin(ompl)
        pilz = {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
            "response_adapters": [
                "default_planning_response_adapters/ValidateSolution"
            ],
        }
        return {
            "robot_description": robot.urdf_xml,
            "robot_description_semantic": robot.srdf_xml,
            "robot_description_kinematics": dict(robot.kinematics),
            "planning_pipelines": {
                "pipeline_names": sorted({pipeline, "ompl", "pilz"})
            },
            "ompl": ompl,
            "pilz": pilz,
            "plan_request_params": {
                "planning_attempts": self.config.planning_attempts,
                "planning_pipeline": pipeline,
                "planner_id": self.config.planner_id,
                "max_velocity_scaling_factor": self.config.velocity_scaling,
                "max_acceleration_scaling_factor": self.config.acceleration_scaling,
                "planning_time": self.config.planning_time_s,
            },
            "planning_scene_monitor_options": {
                "name": f"{self.config.node_name}_scene",
                "robot_description": "robot_description",
                "joint_state_topic": "/unused_joint_states",
                "attached_collision_object_topic": "/unused_attached",
                "publish_planning_scene_topic": "/unused_publish",
                "monitored_planning_scene_topic": "/unused_scene",
                "wait_for_initial_state_timeout": 0.0,
            },
            "trajectory_execution": {"manage_controllers": False},
        }

    def _ensure_robot(self, request: PlanRequest) -> Any:
        model = request.robot
        if self._robot is not None and self._robot_model_digest == model.digest:
            return self._robot
        self.close()
        MoveItPy, _, _ = self._require_native()
        config = self._build_config(request)
        self._robot = MoveItPy(node_name=self.config.node_name, config_dict=config)
        self._robot_model_digest = model.digest
        self._component = self._robot.get_planning_component(
            request.planning_group or model.planning_group
        )
        self._monitor = self._robot.get_planning_scene_monitor()
        self.planning_identity = detect_planner_identity()
        self._identity = self.planning_identity
        return self._robot

    def _state(
        self, request: PlanRequest, q: Sequence[float], rail: float | None = None
    ) -> Any:
        """构造完整 RobotState；声明导轨轴时同时写入导轨坐标。

        只设六轴组位置会让 MoveItPy 把导轨留在默认 0，这正是历史 harness
        的缺口；导轨值必须随请求进入状态，碰撞检查才会用真实基座位置。
        """

        _, _, RobotState = self._require_native()
        values = request.robot.validate_joint_positions(q, name="state")
        with self._monitor.read_only() as scene:
            state = copy.copy(scene.current_state)
        state.set_joint_group_positions(
            request.planning_group or request.robot.planning_group, list(values)
        )
        if request.rail_axis is not None:
            value = request.start_rail_m if rail is None else rail
            if value is None:
                raise ValueError("请求声明了导轨轴却没有导轨坐标")
            state.set_joint_positions([request.rail_axis.axis_id], [float(value)])
        state.update()
        return state

    def _load_world(self, request: PlanRequest) -> None:
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        world = request.collision_world
        scene_key = (
            world.digest,
            tuple(
                (
                    item.body_id,
                    item.link_name,
                    item.flange_from_body,
                    item.touch_links,
                )
                for item in request.attachments
            ),
            request.constraints.allowed_collision_pairs,
        )
        if self._loaded_scene_key == scene_key:
            return
        with self._monitor.read_write() as scene:
            for object_id in self._attached_ids:
                detach = AttachedCollisionObject()
                detach.object.id = object_id
                detach.object.operation = CollisionObject.REMOVE
                scene.process_attached_collision_object(detach)
            self._attached_ids.clear()
            for object_id in self._loaded_object_ids:
                removal = CollisionObject()
                removal.id = object_id
                removal.operation = CollisionObject.REMOVE
                scene.apply_collision_object(removal)
            self._loaded_object_ids.clear()
            for left, right in self._allowed_pairs:
                scene.allowed_collision_matrix.set_entry(left, right, False)
            self._allowed_pairs.clear()
            attachments = {item.body_id: item for item in request.attachments}
            by_id = {item.object_id: item for item in world.objects}
            for obj in world.objects:
                if obj.object_id in attachments:
                    continue
                scene.apply_collision_object(
                    _object_to_native(obj, frame_id=world.frame_id)
                )
                self._loaded_object_ids.add(obj.object_id)
            for body_id, attachment in attachments.items():
                obj = by_id[body_id]
                native = _object_to_native(obj, frame_id=attachment.link_name)
                # AttachedCollisionObject.pose is relative to the attachment
                # link, not world. Use the declared flange_from_body transform.
                native.pose = _pose_matrix_to_message(attachment.flange_from_body)
                attached = AttachedCollisionObject()
                attached.link_name = attachment.link_name
                attached.object = native
                attached.object.operation = CollisionObject.ADD
                attached.touch_links = list(attachment.touch_links)
                scene.process_attached_collision_object(attached)
                self._attached_ids.add(body_id)
            for left, right in request.constraints.allowed_collision_pairs:
                scene.allowed_collision_matrix.set_entry(left, right, True)
                self._allowed_pairs.add((left, right))
        self._loaded_world_digest = world.digest
        self._loaded_scene_key = scene_key

    def _path_constraints(self, request: PlanRequest) -> Any:
        from moveit_msgs.msg import Constraints, OrientationConstraint

        constraints = Constraints()
        for spec in request.constraints.orientation:
            item = OrientationConstraint()
            item.header.frame_id = spec.frame_ref
            item.link_name = spec.link_name
            item.orientation.x = spec.orientation_xyzw[0]
            item.orientation.y = spec.orientation_xyzw[1]
            item.orientation.z = spec.orientation_xyzw[2]
            item.orientation.w = spec.orientation_xyzw[3]
            item.absolute_x_axis_tolerance = spec.absolute_x_axis_tolerance_rad
            item.absolute_y_axis_tolerance = spec.absolute_y_axis_tolerance_rad
            item.absolute_z_axis_tolerance = spec.absolute_z_axis_tolerance_rad
            item.weight = spec.weight
            constraints.orientation_constraints.append(item)
        return constraints

    def _goal_state(self, request: PlanRequest, seed_q: Sequence[float]) -> tuple[Any | None, str | None]:
        goal = request.goal
        if goal.kind == "joint":
            assert goal.joint_positions is not None
            return self._state(request, goal.joint_positions, request.goal_rail_m), None
        assert goal.pose is not None
        group = request.planning_group or request.robot.planning_group
        pose = goal.pose
        seeds = request.seeds or (tuple(seed_q),)
        last_error: str | None = None
        for index, seed in enumerate(seeds):
            state = self._state(request, seed, request.goal_rail_m)
            try:
                solved = state.set_from_ik(
                    group,
                    _pose_matrix_to_message(pose.transform),
                    pose.link_name or request.robot.flange_link,
                    min(float(request.timeout_s), 10.0),
                )
            except Exception as error:  # pragma: no cover - native exception type varies
                last_error = f"ik_exception[{index}]:{type(error).__name__}:{error}"
                continue
            if solved:
                state.update()
                return state, None
            last_error = f"ik_failed[{index}]"
        return None, last_error or "ik_failed"

    def _native_segment(
        self,
        request: PlanRequest,
        start_q: Sequence[float],
        goal: Any,
        *,
        seed_q: Sequence[float],
        timeout_s: float,
    ) -> dict[str, Any]:
        _, PlanRequestParameters, _ = self._require_native()
        component = self._component
        start = self._state(request, start_q, request.start_rail_m)
        start.update()
        component.set_start_state(robot_state=start)
        component.set_goal_state(robot_state=goal)
        component.set_path_constraints(self._path_constraints(request))
        parameters = PlanRequestParameters(self._robot, "")
        parameters.planning_time = timeout_s
        parameters.planning_attempts = self.config.planning_attempts
        parameters.max_velocity_scaling_factor = self.config.velocity_scaling
        parameters.max_acceleration_scaling_factor = self.config.acceleration_scaling
        parameters.planning_pipeline = self.config.pipeline
        parameters.planner_id = self.config.planner_id
        started = time.monotonic()
        try:
            result = component.plan(single_plan_parameters=parameters)
        except Exception as error:
            raise MoveItPlanningError(
                f"MoveIt planning raised {type(error).__name__}: {error}"
            ) from error
        elapsed = time.monotonic() - started
        if not result:
            return {
                "ok": False,
                "status": "unreachable",
                "failure_category": "planner_no_solution",
                "error_code": None,
                "planning_time_s": elapsed,
            }
        code = getattr(result.error_code, "val", None)
        if code != 1:
            failure = describe_moveit_error(code)
            return {
                "ok": False,
                "status": failure.status,
                "failure_category": failure.category,
                "error_code": code,
                "error_name": failure.name,
                "error_message": failure.message,
                "planning_time_s": elapsed,
            }
        trajectory = result.trajectory
        if self.config.pipeline == "ompl":
            try:
                parameterized = trajectory.apply_totg_time_parameterization(
                    self.config.velocity_scaling,
                    self.config.acceleration_scaling,
                    path_tolerance=0.001,
                    resample_dt=0.02,
                    min_angle_change=0.0001,
                )
            except Exception:
                parameterized = False
            if not parameterized:
                return {
                    "ok": False,
                    "status": "invalid",
                    "failure_category": "time_parameterization_failed",
                    "error_code": code,
                    "planning_time_s": elapsed,
                }
        message = trajectory.get_robot_trajectory_msg()
        names = list(message.joint_trajectory.joint_names)
        if names != list(request.robot.joint_names):
            return {
                "ok": False,
                "status": "invalid",
                "failure_category": "trajectory_joint_names_mismatch",
                "error_code": code,
                "planning_time_s": elapsed,
                "actual_joint_names": names,
                "expected_joint_names": list(request.robot.joint_names),
            }
        points = [list(point.positions) for point in message.joint_trajectory.points]
        times = [
            point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            for point in message.joint_trajectory.points
        ]
        if not points or any(len(point) != len(names) for point in points):
            return {
                "ok": False,
                "status": "invalid",
                "failure_category": "empty_or_malformed_trajectory",
                "error_code": code,
                "planning_time_s": elapsed,
            }
        return {
            "ok": True,
            "points": points,
            "times": times,
            "joint_names": list(message.joint_trajectory.joint_names),
            "error_code": code,
            "planning_time_s": elapsed,
        }

    def _collision_state(
        self, request: PlanRequest, q: Sequence[float], rail: float | None = None
    ) -> dict[str, Any]:
        from moveit.core.collision_detection import CollisionRequest, CollisionResult

        state = self._state(request, q, rail)
        collision_request = CollisionRequest()
        collision_request.contacts = True
        collision_request.max_contacts = self.config.collision_contacts
        result = CollisionResult()
        with self._monitor.read_only() as scene:
            scene.check_collision(collision_request, result, state)
        return {
            "collision": bool(result.collision),
            "contacts": [str(key) for key in result.contacts],
        }

    def _dense_check(
        self,
        request: PlanRequest,
        points: Sequence[Sequence[float]],
        rails: Sequence[float] | None = None,
    ) -> tuple[int, dict[str, Any] | None]:
        checks = 0
        rail_values = (
            list(rails)
            if rails is not None
            else [float(request.start_rail_m or 0.0)] * len(points)
        )
        rail_step = float(request.options.get("rail_sample_step_m", 0.01))
        for index, (left, right) in enumerate(zip(points, points[1:])):
            rail_left, rail_right = rail_values[index], rail_values[index + 1]
            intervals = max(
                1,
                math.ceil(
                    max(abs(a - b) for a, b in zip(left, right, strict=True))
                    / self.config.max_joint_sample_step_rad
                ),
                math.ceil(abs(rail_right - rail_left) / rail_step),
            )
            for step in range(intervals + 1):
                q = [
                    a + (b - a) * step / intervals
                    for a, b in zip(left, right, strict=True)
                ]
                rail = rail_left + (rail_right - rail_left) * step / intervals
                checks += 1
                hit = self._collision_state(request, q, rail)
                if hit["collision"]:
                    return checks, {
                        "failure_category": "dense_path_collision",
                        "failed_q": q,
                        "failed_rail_m": rail,
                        "collision": hit,
                    }
        if points:
            checks += 1
            hit = self._collision_state(request, points[-1], rail_values[-1])
            if hit["collision"]:
                return checks, {
                    "failure_category": "goal_collision",
                    "failed_q": list(points[-1]),
                    "failed_rail_m": rail_values[-1],
                    "collision": hit,
                }
        return checks, None

    def plan(self, request: PlanRequest) -> PlanResult:
        """规划一个请求；空成功轨迹会被合同层拒绝，并回显携带的载荷身份。"""

        if not isinstance(request, PlanRequest):
            raise TypeError("request 必须是 PlanRequest")
        return self._plan_body(request).with_payload(request)

    def _plan_body(self, request: PlanRequest) -> PlanResult:
        """规划主体：装载场景、分段调用 MoveItPy 与离散碰撞复核。"""

        if request.constraints.clearance is not None:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "clearance_constraint_not_supported",
                    "reason": "MoveItPy/FCL adapter does not natively enforce the "
                    "requested clearance; refusing rather than silently ignoring it",
                },
            )
        if request.rail_moves:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "rail_motion_not_supported",
                    "reason": "this MoveItPy adapter holds the rail fixed at "
                    "start_rail_m; a rail change needs a separate rail planning "
                    "group (see MoveGroupPlanner) and is refused rather than "
                    "silently planned at rail=0",
                    "start_rail_m": request.start_rail_m,
                    "goal_rail_m": request.goal_rail_m,
                },
            )
        started = time.monotonic()
        group = request.planning_group or request.robot.planning_group
        segments: list[dict[str, Any]] = []
        all_points: list[list[float]] = []
        all_times: list[float] = []
        safe_pose_used = False
        safe = request.constraints.safe_pose
        if safe is not None:
            distance = max(
                abs(left - right)
                for left, right in zip(safe.joint_positions, request.start_q, strict=True)
            )
            if distance > safe.tolerance_rad:
                safe_pose_used = True
        try:
            self._ensure_robot(request)
            self._load_world(request)
            first_goal: Any | None
            if safe_pose_used:
                assert safe is not None
                first_goal = self._state(request, safe.joint_positions)
            elif request.goal.kind == "joint":
                first_goal, error = self._goal_state(request, request.start_q)
                if first_goal is None:
                    return PlanResult(
                        status="unreachable",
                        waypoints=(),
                        planner=self.identity,
                        collision_checks=0,
                        diagnostics={"failure_category": error},
                    )
            else:
                first_goal, error = self._goal_state(request, request.start_q)
                if first_goal is None:
                    return PlanResult(
                        status="unreachable",
                        waypoints=(),
                        planner=self.identity,
                        collision_checks=0,
                        diagnostics={"failure_category": error},
                    )
            segment = self._native_segment(
                request,
                request.start_q,
                first_goal,
                seed_q=request.start_q,
                timeout_s=float(request.timeout_s) / (2.0 if safe_pose_used else 1.0),
            )
            segments.append(segment)
            if not segment["ok"]:
                return PlanResult(
                    status=segment["status"],
                    waypoints=(),
                    planner=self.identity,
                    collision_checks=0,
                    safe_pose_used=safe_pose_used,
                    diagnostics={
                        "failure_category": segment["failure_category"],
                        "moveit_error_code": segment.get("error_code"),
                        "moveit_error_name": segment.get("error_name"),
                        "moveit_error_message": segment.get("error_message"),
                        "planning_time_s": segment.get("planning_time_s"),
                        "segments": segments,
                    },
                )
            all_points.extend(segment["points"])
            all_times.extend(segment["times"])
            if safe_pose_used:
                assert safe is not None
                final_goal, error = self._goal_state(request, safe.joint_positions)
                if final_goal is None:
                    return PlanResult(
                        status="unreachable",
                        waypoints=(),
                        planner=self.identity,
                        collision_checks=0,
                        safe_pose_used=True,
                        diagnostics={"failure_category": error, "segments": segments},
                    )
                second = self._native_segment(
                    request,
                    safe.joint_positions,
                    final_goal,
                    seed_q=safe.joint_positions,
                    timeout_s=float(request.timeout_s) / 2.0,
                )
                segments.append(second)
                if not second["ok"]:
                    return PlanResult(
                        status=second["status"],
                        waypoints=tuple(
                            _waypoints(all_points, all_times, request)
                        ),
                        planner=self.identity,
                        collision_checks=0,
                        safe_pose_used=True,
                        diagnostics={
                            "failure_category": second["failure_category"],
                            "moveit_error_code": second.get("error_code"),
                            "segments": segments,
                        },
                    )
                offset = all_times[-1] + 1e-6 if all_times else 0.0
                for point, timestamp in zip(second["points"], second["times"], strict=True):
                    all_points.append(point)
                    all_times.append(offset + timestamp)
        except PlannerUnavailableError as error:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "planner_unavailable",
                    "error": str(error),
                    "error_type": type(error.__cause__).__name__
                    if error.__cause__ is not None
                    else None,
                },
            )
        except ImportError as error:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "planner_dependency_missing",
                    "error": str(error),
                    "error_type": type(error).__name__,
                },
            )
        except ValueError as error:
            return PlanResult(
                status="invalid",
                waypoints=(),
                planner=self.identity,
                diagnostics={
                    "failure_category": "invalid_planner_configuration",
                    "error": str(error),
                    "error_type": type(error).__name__,
                },
            )
        checks, failure = self._dense_check(request, all_points)
        if failure is not None:
            return PlanResult(
                status="collision",
                waypoints=tuple(_waypoints(all_points, all_times, request)),
                planner=self.identity,
                collision_checks=checks,
                safe_pose_used=safe_pose_used,
                diagnostics={
                    **failure,
                    "segments": segments,
                    "planning_wall_time_s": time.monotonic() - started,
                },
            )
        waypoints = tuple(_waypoints(all_points, all_times, request))
        return PlanResult(
            status="success",
            waypoints=waypoints,
            planner=self.identity,
            collision_checks=checks,
            safe_pose_used=safe_pose_used,
            diagnostics={
                "group": group,
                "joint_names": list(request.robot.joint_names),
                "segments": segments,
                "planning_wall_time_s": time.monotonic() - started,
                "collision_sampling": "discrete_joint_space_not_continuous_proof",
                "executed": False,
                "hardware": False,
            },
        )

    def validate(self, scene: CollisionWorld) -> Mapping[str, Any]:
        """校验碰撞世界值对象；不声称做过碰撞查询。"""

        if not isinstance(scene, CollisionWorld):
            raise TypeError("scene 必须是 CollisionWorld")
        issues: list[dict[str, Any]] = []
        unsupported: list[str] = []
        meshes = 0
        for obj in scene.objects:
            for geometry in obj.geometry:
                if geometry.kind == "mesh":
                    meshes += 1
                    if geometry.mesh_path is not None and not Path(
                        geometry.mesh_path
                    ).is_file():
                        issues.append(
                            {
                                "object_id": obj.object_id,
                                "kind": "missing_mesh",
                                "mesh_path": geometry.mesh_path,
                            }
                        )
                elif geometry.kind not in ("box", "cylinder"):
                    unsupported.append(geometry.kind)
        return {
            "status": "valid" if not issues else "invalid",
            "frame_id": scene.frame_id,
            "object_count": len(scene.objects),
            "mesh_count": meshes,
            "issues": issues,
            "unsupported_geometry": sorted(set(unsupported)),
            "collision_query": "not_run",
            "continuous_proof": False,
        }

    def close(self) -> None:
        """释放 MoveItPy 会话；可重复调用。"""

        robot = self._robot
        self._robot = None
        self._robot_model_digest = None
        self._component = None
        self._monitor = None
        self._loaded_world_digest = None
        self._loaded_scene_key = None
        self._loaded_object_ids.clear()
        self._attached_ids.clear()
        self._allowed_pairs.clear()
        if robot is not None:
            try:
                robot.shutdown()
            except Exception:
                pass

    def __enter__(self) -> MoveItPlanner:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


def _status_for_error_code(code: object) -> PlanStatus:
    """``MoveItErrorCodes`` → 合同状态；完整解释见 ``moveit_error_codes``。"""

    return status_for_moveit_error(code)


def _waypoints(
    points: Sequence[Sequence[float]],
    times: Sequence[float],
    request: PlanRequest,
) -> list[PlanWaypoint]:
    # 导轨段内不动时每个路点都携带同一个显式导轨值；绝不把「未提供」写成 0。
    rail = float(request.start_rail_m) if request.start_rail_m is not None else 0.0
    return [
        PlanWaypoint(tuple(point), rail, float(timestamp), "planner")
        for point, timestamp in zip(points, times, strict=True)
    ]


__all__ = [
    "MoveItPlanner",
    "MoveItPlannerConfig",
    "MoveItPlanningError",
    "PlannerUnavailableError",
    "detect_planner_identity",
]
