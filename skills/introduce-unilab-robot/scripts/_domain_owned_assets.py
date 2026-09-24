"""domain-owned greenfield：vendor URDF/mesh 拷贝与 moveit_model 薄封装。"""

from __future__ import annotations

import hashlib
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _slug_from_urdf(path: Path) -> str:
    root = ET.fromstring(path.read_bytes())
    name = str(root.attrib.get("name") or path.stem).strip()
    match = re.match(r"([a-z0-9]+)_robot$", name, re.IGNORECASE)
    if match:
        return match.group(1).lower()
    token = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    return token or "arm"


def _mesh_names_from_urdf(root: ET.Element) -> tuple[str, ...]:
    names: list[str] = []
    seen: set[str] = set()
    for mesh in root.iter("mesh"):
        filename = str(mesh.attrib.get("filename") or "")
        if "/" in filename:
            filename = filename.rsplit("/", 1)[-1]
        if filename.lower().endswith(".stl") and filename not in seen:
            seen.add(filename)
            names.append(filename)
    if not names:
        raise ValueError("URDF 未引用任何 STL mesh")
    return tuple(names)


def _disabled_collisions_from_links(link_map: dict[str, str]) -> tuple[tuple[str, str, str], ...]:
    """默认相邻 link 碰撞对（canonical 名）。"""

    canonical_links = [link_map.get(f"Link{index}", f"link_{index}") for index in range(1, 7)]
    pairs: list[tuple[str, str, str]] = []
    if "base" in link_map.values() or "base_link" in link_map:
        base = link_map.get("base_link", "base")
        if canonical_links:
            pairs.append((canonical_links[0], base, "Adjacent"))
    for left, right in zip(canonical_links, canonical_links[1:], strict=False):
        pairs.append((left, right, "Adjacent"))
    return tuple(pairs)


def scaffold_domain_owned_assets(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    vendor_urdf: Path,
    vendor_mesh_dir: Path | None,
    apply: bool,
) -> dict[str, Any]:
    domain = domain.resolve()
    vendor_urdf = vendor_urdf.resolve()
    if not vendor_urdf.is_file():
        raise FileNotFoundError(f"vendor URDF 不存在: {vendor_urdf}")

    slug = _slug_from_urdf(vendor_urdf)
    device_dir = domain / domain_pkg / "devices" / device_id
    models_dir = device_dir / "models"
    mesh_dest_dir = models_dir / "meshes" / slug
    urdf_name = vendor_urdf.name
    urdf_dest = models_dir / urdf_name

    digest = sha256_file(vendor_urdf)
    root = ET.fromstring(vendor_urdf.read_bytes())
    mesh_names = _mesh_names_from_urdf(root)

    link_map = {
        "dummy_link": "device_link",
        "base_link": "base",
        **{f"Link{index}": f"link_{index}" for index in range(1, 7)},
    }
    joint_map = {
        "dummy_joint": "base_mount_joint",
        **{f"joint{index}": f"joint_{index}" for index in range(1, 7)},
    }
    disabled = _disabled_collisions_from_links(link_map)
    model_ref = f"package://{domain_pkg}/devices/{device_id}/models/model.yaml"
    moveit_group = f"{slug}_arm"

    model_yaml = f"""schema: unilab.robot-model/v1
model_id: dobot-{slug}
source:
  repository: vendor
  path: {urdf_name}
  sha256: {digest}
kinematic_joints:
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6
joint_state_sources:
  canonical: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  dobot_sdk_v1: [J1, J2, J3, J4, J5, J6]
forbidden_joints:
  - arm_base_joint
moveit_group: {moveit_group}
rviz_required: false
"""

    kinematics_py = _kinematics_py(urdf_name=urdf_name)
    moveit_model_py = _moveit_model_py(
        domain_pkg=domain_pkg,
        device_id=device_id,
        slug=slug,
        urdf_name=urdf_name,
        digest=digest,
        mesh_names=mesh_names,
        link_map=link_map,
        joint_map=joint_map,
        disabled=disabled,
        model_ref=model_ref,
        moveit_group=moveit_group,
    )

    planned = [
        str(urdf_dest.relative_to(domain)).replace("\\", "/"),
        str((models_dir / "model.yaml").relative_to(domain)).replace("\\", "/"),
        str((device_dir / "kinematics.py").relative_to(domain)).replace("\\", "/"),
        str((device_dir / "moveit_model.py").relative_to(domain)).replace("\\", "/"),
    ]
    created: list[str] = []

    if apply:
        models_dir.mkdir(parents=True, exist_ok=True)
        mesh_dest_dir.mkdir(parents=True, exist_ok=True)
        if not urdf_dest.is_file():
            shutil.copy2(vendor_urdf, urdf_dest)
            created.append(planned[0])
        model_yaml_path = models_dir / "model.yaml"
        if not model_yaml_path.is_file():
            model_yaml_path.write_text(model_yaml, encoding="utf-8")
            created.append(planned[1])
        if vendor_mesh_dir is not None:
            vendor_mesh_dir = vendor_mesh_dir.resolve()
            if vendor_mesh_dir.is_dir():
                for name in mesh_names:
                    src = vendor_mesh_dir / name
                    if src.is_file():
                        dest = mesh_dest_dir / name
                        if not dest.is_file():
                            shutil.copy2(src, dest)
        kin_path = device_dir / "kinematics.py"
        if not kin_path.is_file():
            kin_path.write_text(kinematics_py, encoding="utf-8")
            created.append(planned[2])
        moveit_path = device_dir / "moveit_model.py"
        if not moveit_path.is_file():
            moveit_path.write_text(moveit_model_py, encoding="utf-8")
            created.append(planned[3])

    return {
        "slug": slug,
        "source_digest": digest,
        "planned_files": planned,
        "created_files": created,
        "changed": bool(created) if apply else True,
    }


def _kinematics_py(*, urdf_name: str) -> str:
    return f'''"""从 vendor URDF 提供六轴前向运动学。"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from unilab_robot_contracts import (
    CartesianPose,
    JointSpecification,
    JointType,
    RigidTransform,
    ToolContext,
)

_SOURCE_URDF = Path(__file__).resolve().parent / "models" / "{urdf_name}"
_CANONICAL_JOINT_NAMES = tuple(f"joint_{{index}}" for index in range(1, 7))


@dataclass(frozen=True, slots=True)
class _KinematicJoint:
    specification: JointSpecification
    origin: RigidTransform
    axis_xyz: tuple[float, float, float]


def load_joint_specifications() -> tuple[JointSpecification, ...]:
    return tuple(item.specification for item in _load_chain())


def forward_kinematics(
    joint_positions: Sequence[float],
    tool_context: ToolContext,
) -> CartesianPose:
    positions = tuple(float(value) for value in joint_positions)
    chain = _load_chain()
    if len(positions) != len(chain):
        raise ValueError(f"前向运动学必须接收 {{len(chain)}} 个关节值")
    transform = RigidTransform.identity()
    for position, joint in zip(positions, chain, strict=True):
        transform = transform.compose(joint.origin).compose(
            RigidTransform.from_axis_angle(joint.axis_xyz, position)
        )
    tcp = transform.compose(tool_context.mount_to_tcp)
    return CartesianPose("arm_base", tcp.translation_m, tcp.orientation_xyzw)


@lru_cache(maxsize=1)
def _load_chain() -> tuple[_KinematicJoint, ...]:
    root = ET.fromstring(_SOURCE_URDF.read_bytes())
    result: list[_KinematicJoint] = []
    for index, canonical_name in enumerate(_CANONICAL_JOINT_NAMES, start=1):
        element = root.find(f"joint[@name='joint{{index}}']")
        if element is None:
            raise ValueError(f"URDF 缺少 joint{{index}}")
        joint_type = JointType(str(element.attrib.get("type", "")))
        limit = element.find("limit")
        if limit is None:
            raise ValueError(f"joint{{index}} 缺少 limit")
        specification = JointSpecification(
            canonical_name,
            joint_type,
            float(limit.attrib["lower"]),
            float(limit.attrib["upper"]),
        )
        origin = element.find("origin")
        axis = element.find("axis")
        result.append(
            _KinematicJoint(
                specification,
                RigidTransform.from_rpy(
                    _vector(origin, "xyz", (0.0, 0.0, 0.0)),
                    _vector(origin, "rpy", (0.0, 0.0, 0.0)),
                ),
                _vector(axis, "xyz", (0.0, 0.0, 1.0)),
            )
        )
    return tuple(result)


def _vector(
    element: ET.Element | None,
    attribute: str,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    if element is None or attribute not in element.attrib:
        return default
    values = tuple(float(value) for value in element.attrib[attribute].split())
    if len(values) != 3:
        raise ValueError(f"URDF {{attribute}} 必须包含三个数值")
    return values


__all__ = ["forward_kinematics", "load_joint_specifications"]
'''


def _moveit_model_py(
    *,
    domain_pkg: str,
    device_id: str,
    slug: str,
    urdf_name: str,
    digest: str,
    mesh_names: tuple[str, ...],
    link_map: dict[str, str],
    joint_map: dict[str, str],
    disabled: tuple[tuple[str, str, str], ...],
    model_ref: str,
    moveit_group: str,
) -> str:
    mesh_lines = ",\n            ".join(f'"{name}"' for name in mesh_names)
    link_lines = ",\n    ".join(f'"{k}": "{v}"' for k, v in link_map.items())
    joint_lines = ",\n    ".join(f'"{k}": "{v}"' for k, v in joint_map.items())
    disabled_lines = ",\n    ".join(
        f'("{a}", "{b}", "{reason}")' for a, b, reason in disabled
    )
    mesh_path_lines = ",\n        ".join(
        f'_MESH_ROOT / "{name}"' for name in mesh_names
    )
    return f'''"""领域自有 MoveIt 型号（由 introduce-unilab-robot 生成）。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

from unilab_robot_contracts import (
    CartesianPose,
    JointSpecification,
    JointStateNameMap,
    ToolContext,
)
from unilab_robot_model_kit import (
    MoveItModelBundle,
    SixAxisArmModelSpec,
    assemble_six_axis_moveit_model,
    build_joint_state_name_map as kit_build_joint_state_name_map,
)

from .kinematics import forward_kinematics, load_joint_specifications

_SOURCE_DIGEST = "{digest}"
_MODEL_ROOT = Path(__file__).resolve().parent / "models"
_MODEL_DESCRIPTOR_PATH = _MODEL_ROOT / "model.yaml"
_SOURCE_URDF = _MODEL_ROOT / "{urdf_name}"
_MESH_ROOT = _MODEL_ROOT / "meshes" / "{slug}"
_CANONICAL_JOINT_NAMES = tuple(f"joint_{{index}}" for index in range(1, 7))
_LINK_NAMES = {{
    {link_lines}
}}
_JOINT_NAMES = {{
    {joint_lines}
}}
_JOINT_EFFORT = (150.0, 150.0, 150.0, 30.0, 30.0, 30.0)
_DISABLED_COLLISIONS = (
    {disabled_lines}
)
_ARM_SPEC = SixAxisArmModelSpec(
    source_urdf=_SOURCE_URDF,
    expected_source_digest=_SOURCE_DIGEST,
    mesh_paths=tuple(
        {mesh_path_lines}
    ),
    link_names=_LINK_NAMES,
    joint_names=_JOINT_NAMES,
    joint_effort=_JOINT_EFFORT,
    canonical_joint_names=_CANONICAL_JOINT_NAMES,
    flange_frame="tool0",
    last_link="link_6",
    disabled_collisions=_DISABLED_COLLISIONS,
    mock_system_suffix="{slug}",
    model_descriptor_path=_MODEL_DESCRIPTOR_PATH,
)


@dataclass(frozen=True)
class ArmModelDescriptor:
    model_ref: str
    joint_specs: tuple[JointSpecification, ...]
    base_frame: str
    base_link: str
    tip_link: str
    planning_group: str

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(specification.name for specification in self.joint_specs)

    def forward_kinematics(
        self,
        joint_positions: Sequence[float],
        tool_context: ToolContext,
    ) -> CartesianPose:
        return forward_kinematics(joint_positions, tool_context)


MODEL_DESCRIPTOR = ArmModelDescriptor(
    model_ref="{model_ref}",
    joint_specs=load_joint_specifications(),
    base_frame="arm_base",
    base_link="device_link",
    tip_link="tool0",
    planning_group="{moveit_group}",
)


def build_joint_state_name_map(
    *,
    device_id: str,
    source: str = "canonical",
) -> JointStateNameMap:
    return kit_build_joint_state_name_map(
        device_id=device_id,
        canonical_joint_names=_CANONICAL_JOINT_NAMES,
        model_descriptor_path=_MODEL_DESCRIPTOR_PATH,
        source=source,
    )


@lru_cache(maxsize=1)
def _load_joint_state_source_mappings() -> dict[str, dict[str, str]]:
    from unilab_robot_model_kit import load_joint_state_source_mappings

    return load_joint_state_source_mappings(
        str(_MODEL_DESCRIPTOR_PATH.resolve()),
        _CANONICAL_JOINT_NAMES,
    )


def build_moveit_model(
    *,
    device_id: str,
    position: Mapping[str, Any] | None = None,
    rotation: Mapping[str, Any] | None = None,
    mount_yaw_deg: float = 0.0,
) -> MoveItModelBundle:
    return assemble_six_axis_moveit_model(
        _ARM_SPEC,
        device_id=device_id,
        position=position,
        rotation=rotation,
        mount_yaw_deg=mount_yaw_deg,
    )


__all__ = [
    "ArmModelDescriptor",
    "MODEL_DESCRIPTOR",
    "MoveItModelBundle",
    "build_joint_state_name_map",
    "build_moveit_model",
]
'''
