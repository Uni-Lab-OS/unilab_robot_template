"""导轨 @device scaffold（默认 L1 unilab-rail-linear 静态外壳）。"""

from __future__ import annotations

import hashlib
import struct
from pathlib import Path
from typing import Any

from _catalog import template_root

_RAIL_JOINT_DIGEST = "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d"
_DEFAULT_RAIL_STATIC_DIGEST = (
    "b19c65c03a228077fc7466db2ae4fbc686a5eb7ffb36e8ec9806a8db6debe323"
)
_DEFAULT_RAIL_PROVIDER = "unilab_rail_linear.static_layout:build_default_rail"
_TEMPLATE_RAIL_MESH = (
    "packages/unilab-rail-linear/src/unilab_rail_linear/models/meshes/arm_slideway.stl"
)


def default_rail_stl_path(script_dir: Path) -> Path:
    """Template L1 包内 canonical 导轨视觉 mesh。"""

    template = template_root(script_dir)
    if template is None:
        raise FileNotFoundError("找不到 unilab_robot_template 根目录")
    mesh = template / _TEMPLATE_RAIL_MESH
    if not mesh.is_file():
        raise FileNotFoundError(f"缺少 template 默认导轨 mesh: {mesh}")
    return mesh


def demo_rail_stl_path(script_dir: Path) -> Path:
    return default_rail_stl_path(script_dir)


def default_rail_static_digest() -> str:
    return _DEFAULT_RAIL_STATIC_DIGEST


def validate_rail_mesh_stl(mesh_path: Path) -> None:
    """Reject empty or structurally invalid binary STL."""

    payload = mesh_path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"rail mesh STL too small: {mesh_path}")
    triangle_count = struct.unpack("<I", payload[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if triangle_count == 0 or expected_size != len(payload):
        raise ValueError(
            "rail mesh STL header/triangle count invalid "
            f"(triangles={triangle_count}, bytes={len(payload)})"
        )
    digest = hashlib.sha256(payload).hexdigest()
    if digest != _DEFAULT_RAIL_STATIC_DIGEST:
        raise ValueError(
            f"rail mesh digest mismatch: expected {_DEFAULT_RAIL_STATIC_DIGEST}, got {digest}"
        )


def rail_device_py(*, rail_id: str, static_digest: str, domain_pkg: str | None = None) -> str:
    class_name = _class_name(rail_id)
    display = rail_id.replace("_", " ").title()
    post_init_block = ""
    if domain_pkg:
        post_init_block = f"""
    @not_action
    def post_init(self, ros_node: object | None = None) -> None:
        from {domain_pkg}.devices.rail_simulation import simulation_backends

        backend = str(self.config.get("standard_execution_backend", "")).strip()
        if backend not in simulation_backends():
            return
        self._ensure_simulation_driver()
        if ros_node is not None:
            self._simulation_driver.attach_ros(ros_node)

    def _ensure_simulation_driver(self) -> None:
        from {domain_pkg}.devices.rail_simulation import (
            RailSimulationDriver,
            register_rail_simulation_axis,
            simulation_backends,
        )

        backend = str(self.config.get("standard_execution_backend", "")).strip()
        if backend not in simulation_backends():
            raise NotImplementedError
        if self._simulation_driver is None:
            self._simulation_driver = RailSimulationDriver(device_id=self.device_id)
        register_rail_simulation_axis(self.device_id, self._simulation_driver)
"""
    imports = (
        "from typing import Any\n\nfrom unilabos.registry.decorators import device, not_action\n"
        if domain_pkg
        else "from unilabos.registry.decorators import device\n"
    )
    init_tail = (
        "        self._simulation_driver = None\n"
        if domain_pkg
        else ""
    )
    return f'''"""由 introduce-unilab-robot 生成的默认导轨设备。"""

from __future__ import annotations

{imports}

@device(
    id="{rail_id}",
    category=["rail"],
    description="Default SZLab slideway rail (package_static + unilab_rail_linear kinematics)",
    displayname="{display}",
    model={{
        "type": "package_static",
        "provider": "{_DEFAULT_RAIL_PROVIDER}",
        "source_digest": "{static_digest}",
        "joint_state_provider": "unilab_rail_linear:build_kinematic_model",
        "joint_state_source_digest": "{_RAIL_JOINT_DIGEST}",
    }},
)
class {class_name}:
    def __init__(self, device_id: str | None = None, config: dict | None = None, **kwargs: object) -> None:
        self.device_id = device_id or "{rail_id}"
        self.config = dict(config or {{}})
        self.config.update(kwargs)
{init_tail}{post_init_block}'''


def _class_name(device_id: str) -> str:
    parts = [part for part in device_id.split("_") if part]
    if not parts:
        return "RailDevice"
    return "".join(part[:1].upper() + part[1:] for part in parts) + "Device"


def scaffold_rail(
    domain: Path,
    *,
    domain_pkg: str,
    rail_id: str,
    rail_stl: Path,
    apply: bool,
    with_rail_simulation: bool = False,
) -> dict[str, Any]:
    domain = domain.resolve()
    rail_py = domain / domain_pkg / "devices" / f"{rail_id}.py"

    validate_rail_mesh_stl(rail_stl)
    static_digest = default_rail_static_digest()
    planned = [
        str(rail_py.relative_to(domain)).replace("\\", "/"),
    ]
    created: list[str] = []

    if apply and not rail_py.is_file():
        rail_py.parent.mkdir(parents=True, exist_ok=True)
        rail_py.write_text(
            rail_device_py(
                rail_id=rail_id,
                static_digest=static_digest,
                domain_pkg=domain_pkg if with_rail_simulation else None,
            ),
            encoding="utf-8",
        )
        created.append(planned[0])

    return {
        "rail_id": rail_id,
        "rail_mesh_source": str(rail_stl),
        "rail_provider": _DEFAULT_RAIL_PROVIDER,
        "static_digest": static_digest,
        "planned_files": planned,
        "created_files": created,
        "changed": bool(created) if apply else not rail_py.is_file(),
    }
