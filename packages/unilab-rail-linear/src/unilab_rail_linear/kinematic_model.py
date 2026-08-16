"""单轴导轨与机械臂共用的关节命名、拓扑摘要和渲染 URDF。"""

from __future__ import annotations

import hashlib
import json
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .factory import MODEL_DESCRIPTOR, build_joint_state_name_map

_DEVICE_ID = re.compile(r"^[A-Za-z0-9_]+$")
_MODEL_YAML = Path(__file__).resolve().parent / "models" / "model.yaml"
_SOURCE_DIGEST = "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d"
_MODEL_ID = "szlab-linear-rail-a"


@dataclass(frozen=True, slots=True)
class RailKinematicModelBundle:
    """供 OS 投影器和 FE 使用的单轴渲染模型。"""

    render_urdf: str
    source_digest: str
    qualified_joint_names: tuple[str, ...]
    topology_digest: str
    mesh_paths: tuple[Path, ...] = ()
    mount_link: str = ""


def build_kinematic_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
) -> RailKinematicModelBundle:
    """构造 Device 命名空间下的单轴棱柱关节渲染 URDF。

    参数：``device_id`` 是 Graph 实例身份；``position``/``rotation`` 由 Graph
    拥有，渲染 URDF 与 CR5 render 一样不写入世界安装。返回：完全限定关节名、
    拓扑摘要和本地 URDF。异常：非法 Device id 或型号描述符漂移时拒绝。
    安全：只声明导轨自己的一根轴，不并入机械臂关节。
    """

    del position, rotation
    normalized = str(device_id).strip()
    if _DEVICE_ID.fullmatch(normalized) is None:
        raise ValueError("device_id 只能包含英文、数字和下划线")
    actual_digest = hashlib.sha256(_MODEL_YAML.read_bytes()).hexdigest()
    if actual_digest != _SOURCE_DIGEST:
        raise ValueError("导轨 model.yaml 源摘要漂移")

    name_map = build_joint_state_name_map(device_id=normalized)
    qualified = name_map.qualified_joint_names
    prefix = f"{normalized}_"
    base = f"{prefix}{MODEL_DESCRIPTOR.base_link}"
    carriage = f"{prefix}{MODEL_DESCRIPTOR.carriage_link}"
    joint_name = qualified[0]
    lower, upper = MODEL_DESCRIPTOR.travel_m
    velocity = MODEL_DESCRIPTOR.velocity_limit_m_s
    robot = ET.Element("robot", {"name": f"{normalized}_rail"})
    ET.SubElement(robot, "link", {"name": base})
    _append_box_link(
        robot,
        carriage,
        size=(0.32, 0.28, 0.05),
        origin_xyz=(0.0, 0.0, 0.175),
    )
    joint = ET.SubElement(robot, "joint", {"name": joint_name, "type": "prismatic"})
    ET.SubElement(joint, "parent", {"link": base})
    ET.SubElement(joint, "child", {"link": carriage})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "axis", {"xyz": "1 0 0"})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": f"{lower}",
            "upper": f"{upper}",
            "effort": "0",
            "velocity": f"{velocity}",
        },
    )
    return RailKinematicModelBundle(
        render_urdf=ET.tostring(robot, encoding="unicode"),
        source_digest=_SOURCE_DIGEST,
        qualified_joint_names=qualified,
        topology_digest=_topology_digest(
            device_id=normalized,
            source_digest=_SOURCE_DIGEST,
            qualified_joint_names=qualified,
        ),
        mount_link=carriage,
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
            "model": _MODEL_ID,
            "source_digest": source_digest,
            "joint_names": qualified_joint_names,
        },
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _append_box_link(
    robot: ET.Element,
    name: str,
    *,
    size: tuple[float, float, float],
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """为渲染 URDF 追加一个本地 box link，不引用外部 mesh。"""

    link = ET.SubElement(robot, "link", {"name": name})
    visual = ET.SubElement(link, "visual")
    ET.SubElement(
        visual,
        "origin",
        {"xyz": f"{origin_xyz[0]} {origin_xyz[1]} {origin_xyz[2]}", "rpy": "0 0 0"},
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "box",
        {"size": f"{size[0]} {size[1]} {size[2]}"},
    )


__all__ = ["RailKinematicModelBundle", "build_kinematic_model"]
