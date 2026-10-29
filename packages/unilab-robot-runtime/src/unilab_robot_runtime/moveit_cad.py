"""Plan-only MoveIt adapter for a frozen CAD/FCL snapshot.

This module was lifted from the 06 post-process station's planner. The native
MoveItPy/FCL mechanics (scene object loading, mesh conversion, rail-coordinate
shifting, dense collision sampling, response checks) are reusable; the device
package owns request semantics, material custody and response schema.

Runs only inside an environment with MoveItPy installed. The six-axis provider
owns kinematics; the rail remains a separate assembly coordinate and is never
inserted into the six-axis group. No ``execute()`` is called.

The public, engine-agnostic planning surface is
``unilab_robot_contracts.planning.MotionPlanner`` /
``unilab_robot_runtime.moveit_planner.MoveItPlanner``. This module additionally
keeps the established CAD/legacy request path used by the post-process shim.
"""
import gc
import copy
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any


def _finite_joint_vector(value: Any, *, label: str) -> list[float]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{label} 必须是数值序列")
    values = [float(item) for item in value]
    if not values or not all(math.isfinite(item) for item in values):
        raise ValueError(f"{label} 必须是有限数值序列")
    return values


def _default_request_validator(request: Mapping[str, Any], geometry: Mapping[str, Any]) -> None:
    """最小通用校验；设备包必须注入自己的完整请求合同。"""

    if not isinstance(request, Mapping):
        raise TypeError("request 必须是映射")
    if not isinstance(geometry, Mapping):
        raise TypeError("geometry 必须是映射")
    _finite_joint_vector(request.get("start_q"), label="start_q")
    if request.get("mode") not in ("RRTConnect", "LIN", "PTP", "RAIL", "CHECK_ACTUAL"):
        raise ValueError("未知规划模式")
    for key in ("rail_m",):
        if key in request:
            value = request[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key} 必须是有限数")


def _default_scaling(request: Mapping[str, Any]) -> dict[str, float]:
    value = request.get("planning_scaling")
    if isinstance(value, Mapping):
        return {
            "velocity": float(value.get("velocity", 0.2)),
            "acceleration": float(value.get("acceleration", 0.2)),
        }
    return {"velocity": 0.2, "acceleration": 0.2}


def _default_budget(request: Mapping[str, Any]) -> float:
    value = float(request.get("allowed_planning_time", 10.0))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("allowed_planning_time 必须是正有限数")
    return value


def _default_path_constraints(planner: Any, start: Any, request: Mapping[str, Any]) -> dict[str, Any]:
    """默认不施加设备级路径约束，但显式清空上一条请求的约束。"""

    from moveit_msgs.msg import Constraints

    planner.component.set_path_constraints(Constraints())
    return {
        "applied": False,
        "reason": "no device path-constraint hook supplied",
    }


def _default_endpoint_validator(
    request: Mapping[str, Any],
    names: Any,
    points: Any,
    times: Any,
    goal: Any,
    endpoint_world: Any,
    validate_trajectory: Any,
) -> tuple[bool, dict[str, Any]]:
    """通用端点检查：复用轨迹校验，不猜测设备位姿语义。"""

    zero_motion = validate_trajectory(names, points, times, goal, goal)
    return zero_motion, {
        "status": "passed",
        "validator": "generic_joint_endpoint_only",
        "pose_endpoint_checked": False,
    }


def _make_default_trajectory_validator(joint_names: tuple[str, ...]):
    def validate(names: Any, points: Any, times: Any, start: Any, goal: Any, *, simulation_verification_mode: str = "strict") -> bool:
        if simulation_verification_mode not in ("strict", "planner_only"):
            raise ValueError("Unknown simulation verification mode")
        expected = list(joint_names)
        if list(names) != expected:
            raise ValueError("Trajectory joint names/order differ from provider")
        if not points or len(points) != len(times):
            raise ValueError("Trajectory must contain timed states")
        expected_dimension = len(joint_names)
        for values, label in ((start, "start"), (goal, "goal")):
            if len(values) != expected_dimension or not all(math.isfinite(float(v)) for v in values):
                raise ValueError(f"Invalid trajectory {label}")
        for point in points:
            if len(point) != expected_dimension or not all(math.isfinite(float(v)) for v in point):
                raise ValueError("Invalid trajectory joint positions")
        if not all(math.isfinite(float(t)) and t >= 0 for t in times) or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("Trajectory timestamps must be finite and strictly increasing")
        if len(points) == 1:
            if max(abs(a - b) for a, b in zip(start, goal)) > 1e-6:
                raise ValueError("Single-state trajectory must be a zero-motion command")
            return True
        if max(abs(a - b) for a, b in zip(points[0], start)) > 2e-5:
            raise ValueError("Trajectory does not begin at measured start")
        tolerance = 1e-3 if simulation_verification_mode == "planner_only" else 2e-5
        if max(abs(a - b) for a, b in zip(points[-1], goal)) > tolerance:
            raise ValueError("Trajectory does not reach requested goal")
        return False
    return validate


def _default_config_builder(
    model_dir: Path,
    *,
    simulation_joint_acceleration: float,
    planning_group: str,
    joint_names: tuple[str, ...],
) -> dict[str, Any]:
    """从可移植 model_dir 构造最小 MoveItPy 配置。

    设备包应注入自己的完整配置；默认实现只读取 URDF/SRDF/可选
    ``provider.json``，并在缺少 IK 参数时让位姿目标显式失败。
    """

    model_dir = Path(model_dir)
    urdf = (model_dir / "robot-portable.urdf").read_text(encoding="utf-8")
    srdf = (model_dir / "robot.srdf").read_text(encoding="utf-8")
    provider: dict[str, Any] = {}
    provider_path = model_dir / "provider.json"
    if provider_path.is_file():
        provider = json.loads(provider_path.read_text(encoding="utf-8"))
    return {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": provider.get("kinematics", {}),
        "planning_pipelines": {"pipeline_names": ["ompl", "pilz"]},
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "planner_configs": {
                "RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0}
            },
            planning_group: {
                "default_planner_config": "RRTConnectkConfigDefault",
                "planner_configs": ["RRTConnectkConfigDefault"],
                "longest_valid_segment_fraction": 0.002,
            },
        },
        "pilz": {
            "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
            "response_adapters": ["default_planning_response_adapters/ValidateSolution"],
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
            "planning_time": 30.0,
        },
        "planning_scene_monitor_options": {
            "name": "unilab_cad_scene",
            "robot_description": "robot_description",
            "joint_state_topic": "/unused_joint_states",
            "attached_collision_object_topic": "/unused_attached",
            "publish_planning_scene_topic": "/unused_publish",
            "monitored_planning_scene_topic": "/unused_scene",
            "wait_for_initial_state_timeout": 0.0,
        },
        "trajectory_execution": {"manage_controllers": False},
    }


def _infer_model_roles(
    model_dir: Path,
    *,
    planning_group: str | None,
    flange_link: str | None,
    joint_names: tuple[str, ...] | None,
) -> tuple[str, str, tuple[str, ...]]:
    """从冻结 model_dir 的 SRDF/URDF 推导规划组、法兰和关节顺序。"""

    model_dir = Path(model_dir)
    urdf_path = model_dir / "robot-portable.urdf"
    srdf_path = model_dir / "robot.srdf"
    if not urdf_path.is_file() or not srdf_path.is_file():
        raise ValueError("model_dir 必须包含 robot-portable.urdf 与 robot.srdf")
    urdf = ET.parse(urdf_path)
    srdf = ET.parse(srdf_path)
    groups = srdf.findall("group")
    if not groups:
        raise ValueError("SRDF 至少需要一个 planning group")
    group = planning_group or groups[0].attrib.get("name")
    if not group:
        raise ValueError("无法确定 planning group")
    chain = srdf.find(f"group[@name='{group}']/chain")
    base_link = chain.attrib.get("base_link") if chain is not None else None
    tip_link = chain.attrib.get("tip_link") if chain is not None else None
    inferred: list[str] = []
    if base_link and tip_link:
        current = base_link
        by_parent: dict[str, list[Any]] = {}
        for joint in urdf.findall("joint"):
            by_parent.setdefault(joint.find("parent").attrib.get("link"), []).append(joint)
        seen: set[str] = set()
        while current != tip_link:
            candidates = [
                joint
                for joint in by_parent.get(current, [])
                if joint.attrib.get("type") != "fixed"
            ]
            if not candidates:
                break
            joint = candidates[0]
            inferred.append(joint.attrib["name"])
            current = joint.find("child").attrib["link"]
            if current in seen:
                raise ValueError("URDF 链存在环")
            seen.add(current)
    if joint_names is None:
        names = tuple(inferred) or tuple(
            joint.attrib["name"]
            for joint in urdf.findall("joint")
            if joint.attrib.get("type") != "fixed"
        )
    else:
        names = tuple(joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError("无法从 URDF 推导唯一关节序列")
    flange = flange_link or tip_link
    if not flange:
        links = {joint.find("child").attrib["link"] for joint in urdf.findall("joint")}
        flange = sorted(links)[-1] if links else None
    if not flange:
        raise ValueError("无法确定法兰 link")
    return group, flange, names


class MoveItCADPlanner:
    """Keep an FCL scene loaded while requests update measured rigid-body poses.

    ``config``/``config_builder`` 必须给出目标 MoveIt 实例的完整参数。
    设备包可注入请求校验、缩放、时间预算、水平约束、轨迹校验和端点校验
    hooks；未注入时使用保守的通用实现，并显式声明未检查的领域语义。
    """

    def __init__(
        self,
        model_dir: str | Path,
        geometry_file: str | Path,
        *,
        simulation_joint_acceleration: float = 0.5,
        config: Mapping[str, Any] | None = None,
        config_builder: Callable[..., Mapping[str, Any]] | None = None,
        request_validator: Callable[[Mapping[str, Any], Mapping[str, Any]], None] | None = None,
        scaling_resolver: Callable[[Mapping[str, Any]], Mapping[str, float]] | None = None,
        budget_resolver: Callable[[Mapping[str, Any]], float] | None = None,
        level_sampler: Callable[[Any, Mapping[str, Any]], Mapping[str, Any] | None] | None = None,
        path_constraints_builder: Callable[[Any, Any, Mapping[str, Any]], Mapping[str, Any]] | None = None,
        trajectory_validator: Callable[..., bool] | None = None,
        endpoint_validator: Callable[..., tuple[bool, Mapping[str, Any]]] | None = None,
        planning_group: str | None = None,
        flange_link: str | None = None,
        joint_names: tuple[str, ...] | None = None,
        node_name: str = "unilab_cad_planner",
        contact_binding_sha256: str | None = None,
        carriage_object_id: str | None = None,
        carriage_base_link: str | None = None,
        rail_exemption_reason: str = "original_rail_mount",
        timing_schema: str = "unilab.simulation-timing-assumption/v1",
        report_schema: str = "unilab.moveit-cad-path/v1",
        rail_report_schema: str = "unilab.moveit-rail-path/v1",
    ) -> None:
        if (
            isinstance(simulation_joint_acceleration, bool)
            or not math.isfinite(float(simulation_joint_acceleration))
            or float(simulation_joint_acceleration) not in (0.5, 1.0)
        ):
            raise ValueError("Unsupported simulation joint acceleration assumption")
        model_dir = Path(model_dir)
        self.model_dir = model_dir
        group, flange, names = _infer_model_roles(
            model_dir, planning_group=planning_group, flange_link=flange_link, joint_names=joint_names
        )
        self.group = group
        self.flange = flange
        self.joint_names = names
        self.request_validator = request_validator or _default_request_validator
        self.scaling_resolver = scaling_resolver or _default_scaling
        self.budget_resolver = budget_resolver or _default_budget
        self.level_sampler = level_sampler
        self.path_constraints_builder = path_constraints_builder or _default_path_constraints
        self.trajectory_validator = trajectory_validator or _make_default_trajectory_validator(names)
        self.endpoint_validator = endpoint_validator or _default_endpoint_validator
        if config is None:
            builder = config_builder or _default_config_builder
            config = builder(
                model_dir,
                simulation_joint_acceleration=float(simulation_joint_acceleration),
                planning_group=group,
                joint_names=names,
            )
        config = dict(config)
        self.native_config = config
        self.simulation_timing_assumption = {
            "schema": timing_schema,
            "hardware": False,
            "joint_acceleration_rad_s2": float(simulation_joint_acceleration),
            "joint_deceleration_rad_s2": -float(simulation_joint_acceleration),
            "source": "explicit_simulation_assumption_not_manufacturer_limit",
            "native_configuration_sha256": hashlib.sha256(
                json.dumps(config, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest(),
        }
        import numpy as np
        from moveit.planning import MoveItPy

        self.np = np
        self.robot = MoveItPy(node_name=node_name, config_dict=config)
        urdf = ET.parse(model_dir / "robot-portable.urdf")
        self.joint_limits = []
        for name in names:
            limit = urdf.find(f".//joint[@name='{name}']/limit")
            if limit is None:
                raise ValueError(f"URDF 缺少关节限位: {name}")
            self.joint_limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
        self.model = self.robot.get_robot_model()
        self.component = self.robot.get_planning_component(self.group)
        self.monitor = self.robot.get_planning_scene_monitor()
        self.geometry_file = Path(geometry_file)
        self.geometry = json.loads(self.geometry_file.read_text(encoding="utf-8"))
        if self.geometry["status"] != "exported":
            raise ValueError("Incomplete geometry export")
        arrays = self.geometry_file.parent / self.geometry["arrays"]
        if hashlib.sha256(arrays.read_bytes()).hexdigest() != self.geometry["arrays_sha256"]:
            raise ValueError("Geometry array digest differs")
        self.arrays = np.load(arrays, allow_pickle=False)
        self.objects = {}
        self.bodies = {}
        unsupported = [
            s["prim"]
            for b in self.geometry["bodies"]
            for s in b["shapes"]
            if s["type"] not in ("Mesh", "Cube", "Cylinder")
        ]
        if unsupported:
            raise ValueError("Unsupported exact planner geometry: " + str(unsupported))
        self.attached = set()
        self.allowed = []
        self.world_ids = set()
        self.world_poses = {}
        self.geometry_sha256 = hashlib.sha256(self.geometry_file.read_bytes()).hexdigest()
        self.model_sources = {
            str(f.relative_to(model_dir)): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in model_dir.rglob("*")
            if f.is_file()
            and (f.suffix.lower() in (".urdf", ".srdf", ".stl") or f.name == "provider.json")
        }
        self.contact_binding_sha256 = contact_binding_sha256
        self.carriage_object_id = carriage_object_id
        self.carriage_base_link = carriage_base_link
        self.rail_exemption_reason = rail_exemption_reason
        self.report_schema = report_schema
        self.rail_report_schema = rail_report_schema
        self._load_objects()

    def _pose(self,matrix):
        from geometry_msgs.msg import Pose
        m=self.np.asarray(matrix);r=m[:3,:3];trace=float(self.np.trace(r))
        # Stable matrix-to-quaternion conversion without an undeclared scipy dependency.
        if trace>0:
            s=math.sqrt(trace+1)*2
            q=[(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,s/4]
        else:
            i=int(self.np.argmax(self.np.diag(r)));j=(i+1)%3;k=(i+2)%3
            s=math.sqrt(max(0.,1+r[i,i]-r[j,j]-r[k,k]))*2;q=[0.]*4
            q[i]=s/4;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
        p=Pose()
        p.position.x,p.position.y,p.position.z=map(float,m[:3,3])
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
        return p

    def _load_objects(self):
        from .cad_scene import CadScene
        from .moveit_move_group import _object_to_native
        catalog = CadScene(self.geometry_file, expected_sha256=self.geometry_sha256)
        self.bodies.update(catalog.bodies)
        with self.monitor.read_write() as scene:
            for identity, value in catalog.objects.items():
                obj = _object_to_native(value)
                if scene.apply_collision_object(obj) is False:
                    raise RuntimeError("CAD object rejected: " + identity)
                body_path = catalog.object_bodies[identity]
                self.objects[identity] = (obj, body_path)
                self.world_ids.add(identity)
                self.world_poses[identity] = self.np.array(value.world_pose)
        print("CAD_SCENE_READY", len(self.objects), flush=True)

    def _update_scene(self,request):
        from moveit_msgs.msg import CollisionObject,AttachedCollisionObject
        np=self.np;rail=float(request['rail_m'])
        if not 0<=rail<=1:raise ValueError('Rail limit exceeded')
        poses=request.get('body_poses',{})
        attachments={a['body']:a for a in request.get('attachments',[])}
        if len(attachments)!=len(request.get('attachments',[])):raise ValueError('Duplicate attachment')
        required_tools={p for p,b in self.bodies.items() if b['role'] in ('tool_base','tool_finger')}
        if not required_tools.issubset(attachments):raise ValueError('All actual tool bodies must follow the flange')
        payload=set(attachments)-required_tools
        expected_payload=set(request.get('carried_bodies',[]))
        if payload!=expected_payload:raise ValueError('Attached payload differs from custody bundle')
        held=request.get('held_material')
        if bool(payload)!=bool(held):raise ValueError('Payload attachment/custody mismatch')
        if held and not any(self.bodies[p].get('identity')==held for p in payload):
            raise ValueError('Held carrier missing from attached payload')
        unknown=set(attachments)-set(self.bodies)
        if unknown:raise ValueError('Unknown attachment bodies: '+str(unknown))
        with self.monitor.read_write() as scene:
            # Remove old attachments before repopulating/updating world objects.
            for identity in self.attached:
                old=AttachedCollisionObject();old.object.id=identity;old.object.operation=CollisionObject.REMOVE
                scene.process_attached_collision_object(old)
            self.attached=set()
            for name1,name2 in self.allowed:scene.allowed_collision_matrix.set_entry(name1,name2,False)
            self.allowed=[]
            for identity,(original,body_path) in self.objects.items():
                body=self.bodies[body_path]
                matrix=np.array(poses.get(body_path,body['world_from_body_initial']),dtype=float)
                # Fixed provider root stays at rail zero; shift the entire world
                # by -rail. The rail is never inserted in the six-axis group.
                matrix[0,3]-=rail
                if body_path in attachments:
                    attachment=attachments[body_path]
                    removed=CollisionObject();removed.id=identity;removed.operation=CollisionObject.REMOVE
                    scene.apply_collision_object(removed)
                    self.world_ids.discard(identity)
                    original.header.frame_id=self.flange;original.pose=self._pose(attachment['flange_from_body'])
                    original.operation=CollisionObject.ADD
                    attached=AttachedCollisionObject();attached.link_name=self.flange;attached.object=original
                    attached.touch_links=attachment['touch_links']
                    scene.process_attached_collision_object(attached);self.attached.add(identity)
                else:
                    if identity not in self.world_ids:
                        original.header.frame_id='world';original.pose=self._pose(matrix);original.operation=CollisionObject.ADD
                        scene.apply_collision_object(original);self.world_ids.add(identity)
                    elif not np.array_equal(matrix,self.world_poses[identity]):
                        move=CollisionObject();move.id=identity;move.header.frame_id='world'
                        move.pose=self._pose(matrix);move.operation=CollisionObject.MOVE
                        scene.apply_collision_object(move)
                    self.world_poses[identity]=matrix.copy()
            for exception in request.get('allowed_contacts',[]):
                a,b=exception['a'],exception['b']
                if not exception.get('reason') or a not in self.objects or b not in self.objects:
                    raise ValueError('Contact exception must name exact exported objects and reason')
                scene.allowed_collision_matrix.set_entry(a,b,True);self.allowed.append((a,b))
            for exception in request.get('assembly_contacts',[]):
                a,b=exception['a'],exception['b']
                scene.allowed_collision_matrix.set_entry(a,b,True);self.allowed.append((a,b))
            if request.get('contact_transition'):
                carrier=request['contact_transition']['carrier']
                for finger,metadata in self.bodies.items():
                    if metadata['role']=='tool_finger':
                        scene.allowed_collision_matrix.set_entry(carrier,finger,True);self.allowed.append((carrier,finger))

    def state(self,q):
        if len(q)!=len(self.joint_names) or not all(math.isfinite(float(v)) for v in q):
            raise ValueError(f'Expected {len(self.joint_names)} finite arm joints')
        if any(v<lo or v>hi for v,(lo,hi) in zip(q,self.joint_limits)):raise ValueError('Provider joint bound exceeded')
        with self.monitor.read_only() as scene:state=copy.copy(scene.current_state)
        state.set_joint_group_positions(self.group,q);state.update();return state

    def collision(self,state):
        from moveit.core.collision_detection import CollisionRequest,CollisionResult
        request=CollisionRequest();request.contacts=True;request.max_contacts=30
        result=CollisionResult()
        with self.monitor.read_only() as scene:scene.check_collision(request,result,state)
        return dict(collision=result.collision,contacts=[str(k) for k in result.contacts],
            contact_details=[dict(a=k[0],b=k[1],points=[dict(position=list(c.position),normal=list(c.normal),depth=c.depth) for c in values])
                for k,values in result.contacts.items()])

    def _level(self,state,request):
        if self.level_sampler is None:
            return None
        flange=state.get_global_link_transform(self.flange).copy()
        flange[0,3]+=request.get('rail_m',request.get('rail_start_m',0.))
        return self.level_sampler(flange.tolist(),request)

    def carriage_collision(self,state):
        """Include the moving rail body in robot-v-world FCL, without re-meshing.

        Temporary attachment is query-only. The source carriage remains a
        separately driven axis. Only its original CR5 base mount is allowed.
        """
        from moveit_msgs.msg import CollisionObject,AttachedCollisionObject
        # First keep carriage in world: this covers real tool/payload against
        # carriage even if an attached-body implementation suppresses siblings.
        ordinary=self.collision(state)
        if ordinary['collision']:return ordinary
        if self.carriage_object_id is None or self.carriage_base_link is None:
            raise ValueError('Device package did not declare carriage_object_id/carriage_base_link')
        identity=self.carriage_object_id
        if identity not in self.world_ids:raise ValueError('Carriage missing from query world')
        with self.monitor.read_write() as scene:
            scene.current_state.set_joint_group_positions(self.group,state.get_joint_group_positions(self.group))
            scene.current_state.update()
            attached=AttachedCollisionObject();attached.link_name=self.flange
            attached.object.id=identity;attached.object.operation=CollisionObject.ADD;attached.touch_links=[]
            if scene.process_attached_collision_object(attached) is False:raise ValueError('Cannot attach carriage query body')
            self.world_ids.remove(identity)
            for a,b in self.allowed:scene.allowed_collision_matrix.set_entry(a,b,True)
            scene.allowed_collision_matrix.set_entry(identity,self.carriage_base_link,True)
        try:
            # The pre-attachment RobotState does not contain this query body.
            # Copy the scene *after* attachment, retaining all attached shapes.
            with self.monitor.read_only() as scene:query_state=copy.copy(scene.current_state)
            query_state.update()
            return self.collision(query_state)
        finally:
            with self.monitor.read_write() as scene:
                remove=AttachedCollisionObject();remove.object.id=identity;remove.object.operation=CollisionObject.REMOVE
                if scene.process_attached_collision_object(remove) is False:raise ValueError('Cannot restore carriage query body')
                self.world_ids.add(identity)
                scene.allowed_collision_matrix.set_entry(identity,self.carriage_base_link,False)
                for a,b in self.allowed:scene.allowed_collision_matrix.set_entry(a,b,True)

    def _path_constraints(self,start,request):
        """设备包注入的路径约束；返回的字典进入规划回执。"""

        return dict(self.path_constraints_builder(self,start,request))

    def check_actual(self,request):
        samples=request.get('actual_samples',[])
        report=dict(status='failed',hardware=False,mode='CHECK_ACTUAL',
            request_sha256=hashlib.sha256(json.dumps(request,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            geometry_sha256=self.geometry_sha256,model_sources=self.model_sources,
            state_snapshot_id=request['state_snapshot_id'],continuous_collision_proof=False)
        if not samples:return dict(report,failure_category='empty_actual_trajectory')
        self._update_scene(request);maximum_tilt=0.;previous_tick=None
        for index,row in enumerate(samples):
            if previous_tick is not None and row['tick']!=previous_tick+1:
                return dict(report,failure_category='actual_tick_gap',sample_index=index)
            previous_tick=row['tick']
            measured=row.get('measured_snapshot')
            if measured:
                sample=dict(measured,mode='CHECK_ACTUAL')
                self.request_validator(sample,self.geometry);self._update_scene(sample)
            elif abs(row['rail_m']-request['rail_m'])>1e-10:
                moving=dict(request,rail_m=row['rail_m'],body_poses=copy.deepcopy(request['body_poses']))
                if self.carriage_object_id is None:
                    raise ValueError('Device package did not declare carriage_object_id')
                carriage=self.carriage_object_id
                moving['body_poses'][carriage][0][3]+=row['rail_m']-request['rail_m']
                self._update_scene(moving)
            state=self.state(row['q']);hit=self.carriage_collision(state);level=self._level(state,measured or request)
            if hit['collision']:return dict(report,failure_category='actual_trajectory_collision',sample_index=index,collision=hit)
            if level:
                maximum_tilt=max(maximum_tilt,level['tilt_rad'])
                if level['status']!='passed':return dict(report,failure_category='actual_trajectory_tilt',sample_index=index,level=level)
        return dict(report,status='passed',checked_actual_samples=len(samples),
                    maximum_tilt_rad=maximum_tilt,source_plan_sha256=request['source_plan_sha256'],
                    collision_validation=dict(status='passed',sample_count=len(samples),
                        scope='robot_tool_payload_and_carriage_vs_all_CAD',continuous_collision_proof=False))

    def plan_rail(self,request):
        """Validate the separately driven linear rail, holding all six arm axes."""
        np=self.np;start=float(request['rail_start_m']);end=float(request['rail_end_m'])
        count=max(1,math.ceil(abs(end-start)/.001));rows=[]
        report=dict(schema=request.get('rail_report_schema',self.rail_report_schema),status='failed',hardware=False,
            mode='RAIL',algorithm='calibrated_linear_axis_with_FCL_sampled_swept_geometry',
            max_rail_sample_spacing_m=.001,continuous_collision_proof=False,
            geometry_sha256=self.geometry_sha256,model_sources=self.model_sources,
            state_source=request.get('state_source'),state_snapshot_id=request.get('state_snapshot_id'),
            request_sha256=hashlib.sha256(json.dumps(request,sort_keys=True,separators=(',',':')).encode()).hexdigest())
        max_tilt=0.
        for i in range(count+1):
            rail=start+(end-start)*i/count;sample=dict(request,rail_m=rail)
            sample['body_poses']=copy.deepcopy(request['body_poses'])
            if self.carriage_object_id is None:
                raise ValueError('Device package did not declare carriage_object_id')
            carriage=self.carriage_object_id
            if carriage not in sample['body_poses']:raise ValueError('Rail carriage missing from live scene')
            sample['body_poses'][carriage][0][3]+=rail-start
            self._update_scene(sample);state=self.state(request['start_q']);hit=self.carriage_collision(state)
            level=self._level(state,request)
            if level:
                max_tilt=max(max_tilt,level['tilt_rad'])
                if level['status']!='passed':return dict(report,failure_category='rail_payload_tilt',level=level)
            rows.append(dict(rail_m=rail,collision=hit['collision']))
            if hit['collision']:
                report.update(failure_category='rail_sweep_collision',failed_rail_m=rail,collision=hit,samples=rows)
                return report
        report.update(status='passed',samples=rows,arm_q=request['start_q'],
            duration_s=max(.02,abs(end-start)/.05),rail_velocity_limit_m_s=.05)
        report['level_validation']=dict(status='passed' if request.get('level_constraint') else 'not_applicable',
            sample_count=len(rows),maximum_tilt_rad=max_tilt,constraint=request.get('level_constraint'))
        exemptions = []
        if self.carriage_object_id is not None and self.carriage_base_link is not None:
            exemptions.append(dict(a=self.carriage_object_id,b=self.carriage_base_link,
                reason=self.rail_exemption_reason))
        report['collision_validation']=dict(status='passed',sample_count=len(rows),max_rail_step_m=.001,
            continuous_collision_proof=False,scope='robot_tool_payload_and_carriage_vs_all_CAD',
            assembly_exemptions=exemptions)
        return report

    def plan_legacy(self,request):
        """既有的设备请求字典入口；新代码应使用 ``MoveItPlanner.plan``。"""
        from moveit.planning import PlanRequestParameters
        try:self.request_validator(request,self.geometry)
        except (ValueError,TypeError,KeyError) as error:
            return dict(status='failed',hardware=False,failure_category='invalid_request_contract',error=str(error))
        if request['mode']=='RAIL':return self.plan_rail(request)
        if request['mode']=='CHECK_ACTUAL':return self.check_actual(request)
        try:scaling=self.scaling_resolver(request);budget=self.budget_resolver(request)
        except (ValueError,TypeError) as error:
            return dict(status='failed',hardware=False,failure_category='invalid_request_contract',error=str(error))
        self._update_scene(request)
        seed=request.get('goal_q',request['start_q'])
        try:start=self.state(request['start_q']);goal=self.state(seed)
        except ValueError as error:return dict(status='failed',hardware=False,failure_category='joint_bounds_or_shape',error=str(error))
        flange=start.get_global_link_transform(self.flange).copy();flange[0,3]+=request['rail_m']
        for attachment in request['attachments']:
            expected=flange@self.np.array(attachment['flange_from_body'])
            actual=self.np.array(request['body_poses'][attachment['body']])
            if float(self.np.max(abs(expected-actual)))>1e-5:
                return dict(status='failed',hardware=False,failure_category='attachment_pose_inconsistent',body=attachment['body'])
        if 'goal_flange_world' in request:
            matrix=self.np.array(request['goal_flange_world'],dtype=float);matrix[0,3]-=request['rail_m']
            if not goal.set_from_ik(self.group,self._pose(matrix),self.flange,.2):
                return dict(status='failed',hardware=False,failure_category='fixed_rail_goal_ik_failed')
            goal.update()
        solved=goal.get_joint_group_positions(self.group).tolist()
        try:goal=self.state(solved)
        except ValueError as error:return dict(status='failed',hardware=False,failure_category='ik_solution_outside_provider_limits',error=str(error),solved_q=solved)
        report=dict(schema=request.get('report_schema',self.report_schema),status='failed',hardware=False,
            geometry_sha256=self.geometry_sha256,rail_m=request['rail_m'],mode=request['mode'],
            request_sha256=hashlib.sha256(json.dumps(request,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            model_sources=self.model_sources,state_source=request.get('state_source'),state_snapshot_id=request.get('state_snapshot_id'),
            source='actual_MoveIt_OMPL_or_Pilz',collision_geometry='authored_CAD_meshes_not_PhysX_cooked',
            allowed_contacts=request.get('allowed_contacts',[]),attachments=request.get('attachments',[]))
        report.update(assembly_contacts=request.get('assembly_contacts',[]),contact_binding_sha256=self.contact_binding_sha256)
        report.update(ik_seed_q=seed,solved_goal_q=solved,planning_scaling=scaling,allowed_planning_time=budget)
        if 'goal_flange_world' in request:
            actual=goal.get_global_link_transform(self.flange).copy();actual[0,3]+=request['rail_m']
            wanted=self.np.array(request['goal_flange_world'])
            report['goal_fk_residual']=dict(position_m=float(self.np.linalg.norm(actual[:3,3]-wanted[:3,3])),
                rotation_matrix=float(self.np.max(abs(actual[:3,:3]-wanted[:3,:3]))))
        for label,state in [('start',start),('goal',goal)]:
            result=self.carriage_collision(state) if label=='start' else self.collision(state);report[label+'_collision']=result
            if result['collision']:report['failure_category']=label+'_collision';return report
            level=self._level(state,request)
            if level and level['status']!='passed':return dict(report,failure_category=label+'_payload_tilt',level=level)
        report['orientation_path_constraint']=self._path_constraints(start,request)
        goal_world=goal.get_global_link_transform(self.flange).copy();goal_world[0,3]+=request['rail_m']
        report['goal_flange_world_actual']=goal_world.tolist()
        self.component.set_start_state(robot_state=start);self.component.set_goal_state(robot_state=goal)
        parameters=PlanRequestParameters(self.robot,'')
        parameters.planning_time=budget;parameters.planning_attempts=1
        parameters.max_velocity_scaling_factor=scaling['velocity'];parameters.max_acceleration_scaling_factor=scaling['acceleration']
        if request['mode'] in ('LIN','PTP'):parameters.planning_pipeline='pilz';parameters.planner_id=request['mode']
        elif request['mode']=='RRTConnect':parameters.planning_pipeline='ompl';parameters.planner_id='RRTConnectkConfigDefault'
        else:raise ValueError('Unsupported real planner mode')
        report.update(native_planning_pipeline=parameters.planning_pipeline,native_planner_id=parameters.planner_id)
        solve_started=time.monotonic();result=self.component.plan(single_plan_parameters=parameters)
        report['planner_wall_time_s']=time.monotonic()-solve_started
        if not result:report['failure_category']='planner_no_solution';return report
        code=getattr(result.error_code,'val',None)
        report['moveit_error_code']=code
        if code!=1:report['failure_category']='moveit_non_success_response';return report
        if request['mode']=='RRTConnect' and not result.trajectory.apply_totg_time_parameterization(scaling['velocity'],scaling['acceleration'],path_tolerance=.001,resample_dt=.02,min_angle_change=.0001):
            report['failure_category']='time_parameterization_failed';return report
        message=result.trajectory.get_robot_trajectory_msg();points=[list(p.positions) for p in message.joint_trajectory.points]
        times=[p.time_from_start.sec+p.time_from_start.nanosec/1e9 for p in message.joint_trajectory.points]
        try:
            def endpoint_world(q):
                actual=self.state(q).get_global_link_transform(self.flange).copy()
                actual[0,3]+=request['rail_m']
                return actual
            zero_motion,report['endpoint_validation']=self.endpoint_validator(request,
                list(message.joint_trajectory.joint_names),points,times,
                goal.get_joint_group_positions(self.group).tolist(),endpoint_world,self.trajectory_validator)
            if zero_motion:
                actual=self.state(points[0]).get_global_link_transform(self.flange).copy()
                actual[0,3]+=request['rail_m']
                wanted=self.np.array(request.get('goal_flange_world',goal_world))
                report['goal_fk_residual']=dict(position_m=float(self.np.linalg.norm(actual[:3,3]-wanted[:3,3])),
                    rotation_matrix=float(self.np.max(abs(actual[:3,:3]-wanted[:3,:3]))))
                if any(not math.isfinite(value) or value>1e-6 for value in report['goal_fk_residual'].values()):
                    raise ValueError('Zero-motion goal pose differs from requested pose')
        except ValueError as error:
            report.update(failure_category='invalid_trajectory_response',error=str(error));return report
        # Check finer samples than returned waypoint spacing with the same CAD FCL scene.
        checks=0;linear_error=0.;orientation_error=0.;max_tilt=0.
        if zero_motion:
            max_tilt=max((self._level(state,request) or {}).get('tilt_rad',0.) for state in (start,goal))
        start_transform=start.get_global_link_transform(self.flange);end_transform=goal.get_global_link_transform(self.flange)
        line=end_transform[:3,3]-start_transform[:3,3];length2=float(line@line)
        for a,b in zip(points,points[1:]):
            intervals=max(1,math.ceil(max(abs(y-x) for x,y in zip(a,b))/.002))
            for i in range(intervals+1):
                q=[x+(y-x)*i/intervals for x,y in zip(a,b)]
                state=self.state(q);hit=self.collision(state);checks+=1
                level=self._level(state,request)
                if level:
                    max_tilt=max(max_tilt,level['tilt_rad'])
                    if level['status']!='passed':return dict(report,failure_category='dense_payload_tilt',failed_q=q,level=level)
                if hit['collision']:
                    report.update(failure_category='dense_path_collision',failed_q=q,collision=hit);return report
                if request['mode']=='LIN':
                    transform=state.get_global_link_transform(self.flange)
                    fraction=float((transform[:3,3]-start_transform[:3,3])@line/length2) if length2>1e-16 else 0.
                    closest=start_transform[:3,3]+line*min(1.,max(0.,fraction))
                    linear_error=max(linear_error,float(self.np.linalg.norm(transform[:3,3]-closest)))
                    orientation_error=max(orientation_error,float(self.np.max(abs(transform[:3,:3]-start_transform[:3,:3]))))
        if request['mode']=='LIN' and (linear_error>1e-4 or orientation_error>1e-3):
            report.update(failure_category='cartesian_straight_segment_deviation',linear_error_m=linear_error,
                orientation_matrix_error=orientation_error);return report
        report.update(status='passed',joint_names=list(message.joint_trajectory.joint_names),positions=points,
            times_s=times,zero_motion=zero_motion,
            dense_collision_samples=checks,max_joint_validation_step_rad=.002,
            linear_error_m=linear_error if request['mode']=='LIN' else None)
        if zero_motion and times[0]>0:
            report.update(stationary_hold=True,stationary_duration_s=times[0])
        report['level_validation']=dict(status='passed' if request.get('level_constraint') else 'not_applicable',
            sample_count=2 if zero_motion else checks,maximum_tilt_rad=max_tilt,constraint=request.get('level_constraint'))
        report['collision_validation']=dict(status='passed',sample_count=2 if zero_motion else checks,max_joint_step_rad=.002,
            continuous_collision_proof=False,scope='start_goal_zero_motion' if zero_motion else 'robot_tool_payload_vs_all_CAD')
        return report

    def close(self):
        self.arrays.close();self.objects.clear();self.component=None;self.monitor=None;self.model=None
        gc.collect();self.robot.shutdown();self.robot=None;gc.collect()
