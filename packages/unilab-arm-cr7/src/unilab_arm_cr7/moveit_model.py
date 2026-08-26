"""从 CR7 distribution 固有资产构造可命名空间化的六轴 MoveIt 模型。"""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from unilab_robot_contracts import JointStateNameMap

from .adapters.moveit import CR7_JOINT_NAMES

_DEVICE_ID = re.compile(r"^[A-Za-z0-9_]+$")
_SOURCE_DIGEST = "c64ced0cdcb2654dc07190dc0b9c4d3db813a9515507a69c5c027d82547c14c7"
_MODEL_ROOT = Path(__file__).resolve().parent / "models"
_MODEL_DESCRIPTOR = _MODEL_ROOT / "model.yaml"
_SOURCE_URDF = _MODEL_ROOT / "cr7_robot.urdf"
_MESH_ROOT = _MODEL_ROOT / "meshes" / "cr7"
_FLANGE_FRAME = "cr7_tool0"
_LINK_NAMES = {
    "dummy_link": "device_link",
    "base_link": "cr7_base",
    **{f"Link{index}": f"cr7_link_{index}" for index in range(1, 7)},
}
_JOINT_NAMES = {
    "dummy_joint": "cr7_base_mount_joint",
    **{f"joint{index}": f"cr7_joint_{index}" for index in range(1, 7)},
}
_JOINT_EFFORT = (150.0, 150.0, 150.0, 30.0, 30.0, 30.0)
_DISABLED_COLLISIONS = (
    ("cr7_base", "cr7_link_1", "Adjacent"),
    ("cr7_base", "cr7_link_2", "Never"),
    ("cr7_base", "cr7_link_4", "Never"),
    ("cr7_link_1", "cr7_link_2", "Adjacent"),
    ("cr7_link_1", "cr7_link_4", "Never"),
    ("cr7_link_2", "cr7_link_3", "Adjacent"),
    ("cr7_link_3", "cr7_link_4", "Adjacent"),
    ("cr7_link_4", "cr7_link_5", "Adjacent"),
    ("cr7_link_4", "cr7_link_6", "Never"),
    ("cr7_link_5", "cr7_link_6", "Adjacent"),
)


@dataclass(frozen=True, slots=True)
class MoveItModelBundle:
    """执行与渲染共享命名/拓扑、但隔离世界安装的六轴模型。"""

    execution_urdf: str
    render_urdf: str
    srdf: str
    ros2_controllers: dict[str, Any]
    moveit_controllers: dict[str, Any]
    kinematics: dict[str, Any]
    joint_limits: dict[str, Any]
    source_digest: str
    mesh_paths: tuple[Path, ...]
    qualified_joint_names: tuple[str, ...]
    topology_digest: str
    rviz_required: bool = False

    @property
    def urdf(self) -> str:
        """为现有 MoveIt Launch 调用方保留执行 URDF 别名。"""

        return self.execution_urdf


def build_joint_state_name_map(
    *,
    device_id: str,
    source: str = "canonical",
) -> JointStateNameMap:
    """按型号包已验证 source 构造 CR7 exact 反馈映射。"""

    raw_to_canonical = _load_joint_state_source_mappings().get(str(source))
    if raw_to_canonical is None:
        raise ValueError(f"CR7 型号包未验证 joint-state source: {source}")
    return JointStateNameMap(
        device_id=str(device_id).strip(),
        canonical_joint_names=CR7_JOINT_NAMES,
        raw_to_canonical=raw_to_canonical,
    )


@lru_cache(maxsize=1)
def _load_joint_state_source_mappings() -> dict[str, dict[str, str]]:
    """从型号描述符加载受 digest 约束的原始关节名白名单。"""

    data = yaml.safe_load(_MODEL_DESCRIPTOR.read_text(encoding="utf-8")) or {}
    canonical = tuple(str(value) for value in data.get("kinematic_joints", ()))
    if canonical != CR7_JOINT_NAMES:
        raise ValueError("CR7 model.yaml 的 kinematic_joints 与型号实现漂移")
    sources = data.get("joint_state_sources")
    if not isinstance(sources, Mapping) or not sources:
        raise ValueError("CR7 model.yaml 缺少 joint_state_sources")
    mappings: dict[str, dict[str, str]] = {}
    for source, raw_names in sources.items():
        names = tuple(str(value) for value in raw_names)
        if len(names) != len(canonical) or len(set(names)) != len(names):
            raise ValueError(f"CR7 joint-state source 非 exact 六轴: {source}")
        mappings[str(source)] = dict(zip(names, canonical, strict=True))
    return mappings


def build_moveit_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    mount_yaw_deg: float = 0.0,
) -> MoveItModelBundle:
    """构造带 Device 命名空间和世界安装位姿的 CR7 MoveIt 模型。

    参数：``device_id`` 是 Graph 实例身份；``position`` 以毫米给出世界位置；
    ``rotation`` 以弧度给出 XYZ 欧拉角；``mount_yaw_deg`` 是导轨滑座上绕 Z
    的固定安装偏航（度），写入 ``type=fixed`` 关节，运行时不变。返回：六轴
    URDF/SRDF、mock 控制器和 MoveIt 参数。异常：非法 Device id、源 URDF 摘要
    漂移、mesh 缺失或安装偏航非有限值时拒绝。
    安全：模型只包含 CR7 六个旋转关节；安装偏航不是第七轴，也不创建导轨轴、
    RViz 或硬件安全许可。
    """

    normalized_device_id = str(device_id).strip()
    if not _DEVICE_ID.fullmatch(normalized_device_id):
        raise ValueError("CR7 MoveIt device_id 只能包含英文、数字和下划线")
    source_bytes = _SOURCE_URDF.read_bytes()
    source_digest = hashlib.sha256(source_bytes).hexdigest()
    if source_digest != _SOURCE_DIGEST:
        raise ValueError("CR7 固有 URDF 摘要漂移")
    mesh_paths = tuple(
        _MESH_ROOT / name
        for name in ("base_link0.STL", *(f"J{index}.STL" for index in range(1, 7)))
    )
    missing_meshes = tuple(path.name for path in mesh_paths if not path.is_file())
    if missing_meshes:
        raise ValueError("CR7 mesh 资产缺失: " + ", ".join(missing_meshes))
    normalized_yaw_deg = _normalize_mount_yaw_deg(mount_yaw_deg)

    prefix = f"{normalized_device_id}_"
    render_root = ET.fromstring(source_bytes)
    render_root.set("name", f"{normalized_device_id}_cr7")
    _qualify_robot_tree(render_root, prefix=prefix, mesh_paths=mesh_paths)
    _append_flange_frame(render_root, prefix=prefix)
    execution_root = deepcopy(render_root)
    _rewrite_render_mesh_uris(
        render_root,
        device_id=normalized_device_id,
    )
    _insert_fixed_mount_yaw(
        execution_root,
        prefix=prefix,
        mount_yaw_deg=normalized_yaw_deg,
    )
    _insert_fixed_mount_yaw(
        render_root,
        prefix=prefix,
        mount_yaw_deg=normalized_yaw_deg,
    )
    execution_root.insert(0, _world_mount_joint(prefix, position, rotation))
    _append_mock_ros2_control(execution_root, prefix=prefix)

    name_map = build_joint_state_name_map(device_id=normalized_device_id)
    qualified_joints = name_map.qualified_joint_names
    topology_digest = _topology_digest(
        device_id=normalized_device_id,
        source_digest=source_digest,
        qualified_joint_names=qualified_joints,
    )
    planning_group = f"{prefix}cr7_arm"
    controller_name = f"{prefix}cr7_controller"
    return MoveItModelBundle(
        execution_urdf=ET.tostring(execution_root, encoding="unicode"),
        render_urdf=ET.tostring(render_root, encoding="unicode"),
        srdf=_build_srdf(prefix=prefix, planning_group=planning_group),
        ros2_controllers=_ros2_controllers(
            controller_name=controller_name,
            joint_names=qualified_joints,
        ),
        moveit_controllers=_moveit_controllers(
            controller_name=controller_name,
            joint_names=qualified_joints,
        ),
        kinematics={
            planning_group: {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
            }
        },
        joint_limits={
            "joint_limits": {
                name: {
                    "has_velocity_limits": True,
                    "max_velocity": 3.14,
                    "has_acceleration_limits": False,
                    "max_acceleration": 0.0,
                }
                for name in qualified_joints
            }
        },
        source_digest=source_digest,
        mesh_paths=mesh_paths,
        qualified_joint_names=qualified_joints,
        topology_digest=topology_digest,
    )


def _topology_digest(
    *,
    device_id: str,
    source_digest: str,
    qualified_joint_names: tuple[str, ...],
) -> str:
    """生成不受安装位姿和本地路径影响的运动学拓扑摘要。"""

    payload = json.dumps(
        {
            "device_id": device_id,
            "model": "cr7",
            "source_digest": source_digest,
            "joint_names": qualified_joint_names,
            "flange_frame": _FLANGE_FRAME,
        },
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _append_flange_frame(root: ET.Element, *, prefix: str) -> None:
    """在第六轴末端增加可按 Device 唯一限定的规范法兰 frame。"""

    qualified_flange = f"{prefix}{_FLANGE_FRAME}"
    ET.SubElement(root, "link", {"name": qualified_flange})
    joint = ET.SubElement(
        root,
        "joint",
        {"name": f"{qualified_flange}_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": f"{prefix}cr7_link_6"})
    ET.SubElement(joint, "child", {"link": qualified_flange})


def _qualify_robot_tree(
    root: ET.Element,
    *,
    prefix: str,
    mesh_paths: tuple[Path, ...],
) -> None:
    """原位限定厂家 URDF 名称并把 mesh URI 收敛到 distribution 内。"""

    mesh_by_name = {path.name: path for path in mesh_paths}
    for link in root.findall("link"):
        link.set("name", f"{prefix}{_LINK_NAMES[link.attrib['name']]}")
    for joint in root.findall("joint"):
        original_joint = joint.attrib["name"]
        joint.set("name", f"{prefix}{_JOINT_NAMES[original_joint]}")
        for relation in ("parent", "child"):
            element = joint.find(relation)
            if element is None:
                raise ValueError(f"CR7 URDF joint 缺少 {relation}")
            element.set(
                "link",
                f"{prefix}{_LINK_NAMES[element.attrib['link']]}",
            )
        if original_joint.startswith("joint"):
            index = int(original_joint.removeprefix("joint")) - 1
            limit = joint.find("limit")
            if limit is None:
                raise ValueError("CR7 URDF 旋转关节缺少 limit")
            limit.set("effort", str(_JOINT_EFFORT[index]))
            limit.set("velocity", "3.14")
    for mesh in root.findall(".//mesh"):
        name = Path(mesh.attrib["filename"]).name
        path = mesh_by_name.get(name)
        if path is None:
            raise ValueError(f"CR7 URDF 引用了未锁定 mesh: {name}")
        mesh.set("filename", path.resolve().as_uri())


def _rewrite_render_mesh_uris(root: ET.Element, *, device_id: str) -> None:
    """把渲染 mesh 改为相对实例 URL，执行 URDF 继续使用本机路径。"""

    for mesh in root.findall(".//mesh"):
        name = Path(mesh.attrib["filename"]).name
        mesh.set("filename", f"{device_id}/meshes/{name}")


def _world_mount_joint(
    prefix: str,
    position: Mapping[str, Any] | None,
    rotation: Mapping[str, Any] | None,
) -> ET.Element:
    """创建 world 到固定安装偏航连杆的固定安装关节。"""

    xyz = _vector(position, scale=0.001)
    rpy = _vector(rotation, scale=1.0)
    joint = ET.Element(
        "joint",
        {"name": f"{prefix}world_mount_joint", "type": "fixed"},
    )
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": "world"})
    ET.SubElement(joint, "child", {"link": f"{prefix}mount_yaw_link"})
    return joint


def _normalize_mount_yaw_deg(value: object) -> float:
    """把导轨滑座上的 Z 安装偏航规范为有限角度（度）。"""

    yaw = float(value)
    if not math.isfinite(yaw):
        raise ValueError("mount_yaw_deg 必须是有限角度（度）")
    return yaw


def _insert_fixed_mount_yaw(
    root: ET.Element,
    *,
    prefix: str,
    mount_yaw_deg: float,
) -> None:
    """插入绕 Z 的固定安装关节；该角运行时不变，不是第七轴。"""

    yaw_link = f"{prefix}mount_yaw_link"
    ET.SubElement(root, "link", {"name": yaw_link})
    joint = ET.SubElement(
        root,
        "joint",
        {"name": f"{prefix}mount_yaw_joint", "type": "fixed"},
    )
    ET.SubElement(
        joint,
        "origin",
        {"xyz": "0 0 0", "rpy": f"0 0 {math.radians(mount_yaw_deg)}"},
    )
    ET.SubElement(joint, "parent", {"link": yaw_link})
    ET.SubElement(joint, "child", {"link": f"{prefix}device_link"})


def _vector(value: Mapping[str, Any] | None, *, scale: float) -> str:
    """把可选 Graph XYZ 映射规范化为 URDF 三元字符串。"""

    candidate: object = value or {}
    if isinstance(candidate, Mapping) and isinstance(candidate.get("position"), Mapping):
        candidate = candidate["position"]
    mapping = candidate if isinstance(candidate, Mapping) else {}
    return " ".join(str(float(mapping.get(axis, 0.0)) * scale) for axis in "xyz")


def _append_mock_ros2_control(root: ET.Element, *, prefix: str) -> None:
    """追加 simulation-only mock_components 六轴控制合同。"""

    control = ET.SubElement(
        root,
        "ros2_control",
        {"name": f"{prefix}cr7_mock_system", "type": "system"},
    )
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"
    for joint_name in CR7_JOINT_NAMES:
        joint = ET.SubElement(control, "joint", {"name": f"{prefix}{joint_name}"})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(state, "param", {"name": "initial_value"}).text = "0.0"
        ET.SubElement(joint, "state_interface", {"name": "velocity"})


def _build_srdf(*, prefix: str, planning_group: str) -> str:
    """生成只包含 CR7 六轴链的 SRDF。"""

    root = ET.Element("robot", {"name": f"{prefix}cr7"})
    group = ET.SubElement(root, "group", {"name": planning_group})
    ET.SubElement(
        group,
        "chain",
        {
            "base_link": f"{prefix}device_link",
            "tip_link": f"{prefix}{_FLANGE_FRAME}",
        },
    )
    for left, right, reason in _DISABLED_COLLISIONS:
        ET.SubElement(
            root,
            "disable_collisions",
            {
                "link1": f"{prefix}{left}",
                "link2": f"{prefix}{right}",
                "reason": reason,
            },
        )
    return ET.tostring(root, encoding="unicode")


def _ros2_controllers(
    *, controller_name: str, joint_names: tuple[str, ...]
) -> dict[str, Any]:
    """生成 controller_manager 使用的完全限定控制器参数。

    ``open_loop_control`` 对 mock 硬件是必需的：没有真实状态闭环时，
    JointTrajectoryController 会把已发出的轨迹判成 ABORTED，RViz 仍能播放
    规划轨迹，卡片却报设备动作失败。
    """

    return {
        "controller_manager": {
            "ros__parameters": {
                controller_name: {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        controller_name: {
            "ros__parameters": {
                "joints": list(joint_names),
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
            }
        },
    }


def _moveit_controllers(
    *, controller_name: str, joint_names: tuple[str, ...]
) -> dict[str, Any]:
    """生成 MoveIt simple controller manager 参数。"""

    return {
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "moveit_simple_controller_manager": {
            "controller_names": [controller_name],
            controller_name: {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": list(joint_names),
            },
        },
    }


__all__ = [
    "MoveItModelBundle",
    "build_joint_state_name_map",
    "build_moveit_model",
]
