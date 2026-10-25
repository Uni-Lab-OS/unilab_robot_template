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

import yaml
from unilab_robot_contracts import (
    ModelProviderDescriptor,
    RigidTransform,
    model_provider_digest,
)

from .factory import MODEL_DESCRIPTOR, MODULE_VERSION, build_joint_state_name_map

_DEVICE_ID = re.compile(r"^[A-Za-z0-9_]+$")
_MODEL_YAML = Path(__file__).resolve().parent / "models" / "model.yaml"
MODEL_YAML_PATH = _MODEL_YAML
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
    安全：只声明导轨自己的一根轴，不并入机械臂关节。滑座
    ``mount_link`` 必须保留为空 link：它是机械臂挂载点，外观由静态外壳
    STL 负责，不得再放无材质占位盒（urdf-loader / RViz 会画成红方块）。
    """

    del position, rotation
    normalized = str(device_id).strip()
    if _DEVICE_ID.fullmatch(normalized) is None:
        raise ValueError("device_id 只能包含英文、数字和下划线")
    actual_digest = _source_file_digest(_MODEL_YAML)
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
    ET.SubElement(robot, "link", {"name": carriage})
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


def describe_model_provider(
    *,
    descriptor_path: str | Path | None = None,
) -> ModelProviderDescriptor:
    """返回导轨型号 provider 的版本、摘要、单轴顺序、行程和 frame。

    参数：``descriptor_path`` 是可选显式输入，默认使用 distribution 内
    锁定的 ``model.yaml``；测试或派生型号可传入临时文件。返回：不含时间、
    随机数或本机路径影响的冻结 descriptor。异常：型号标识、轴名、行程或
    速度与型号实现漂移时拒绝。
    安全：摘要只覆盖 ``model.yaml`` 内容；TCP 取滑座挂载面上的 identity，
    导轨没有工具 TCP。
    """

    yaml_path = _MODEL_YAML if descriptor_path is None else Path(descriptor_path)
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    model_id = str(data.get("model_id", "")).strip()
    axis = data.get("axis") or {}
    axis_name = str(axis.get("name", "")).strip()
    travel = tuple(float(value) for value in axis.get("travel_m", ()))
    velocity = float(axis.get("velocity_limit_m_s", float("nan")))
    if not model_id:
        raise ValueError("导轨 model.yaml 缺少 model_id")
    if (
        model_id != _MODEL_ID
        or axis_name != MODEL_DESCRIPTOR.axis_joint
        or travel != MODEL_DESCRIPTOR.travel_m
        or velocity != MODEL_DESCRIPTOR.velocity_limit_m_s
    ):
        raise ValueError("导轨 model.yaml 与型号实现漂移")
    tcp = RigidTransform.identity()
    digest = model_provider_digest(
        name=model_id,
        version=MODULE_VERSION,
        joint_names=(axis_name,),
        joint_limits=(travel,),
        tcp=tcp,
        sources={"model": _source_file_digest(yaml_path)},
    )
    return ModelProviderDescriptor(
        name=model_id,
        version=MODULE_VERSION,
        digest=digest,
        joint_names=(axis_name,),
        joint_limits=(travel,),
        tcp=tcp,
        model_path=str(yaml_path),
        joint_units=("m",),
        base_frame=MODEL_DESCRIPTOR.base_link,
        tip_frame=MODEL_DESCRIPTOR.carriage_link,
    )


def _source_file_digest(path: Path) -> str:
    """对型号描述符做换行无关摘要。

    参数：``path`` 是 ``model.yaml``。返回：规范化为 LF 后的 SHA-256。
    异常：文件不可读时传播 ``OSError``。安全：Windows CRLF 检出不得被当成
    型号漂移；内容变化仍会改变摘要。
    """

    normalized = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(normalized).hexdigest()


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


__all__ = [
    "MODEL_YAML_PATH",
    "RailKinematicModelBundle",
    "build_kinematic_model",
    "describe_model_provider",
]
