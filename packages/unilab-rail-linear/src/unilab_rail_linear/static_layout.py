"""SZLab 同源默认导轨静态外壳（package_static Provider）。"""

from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace

_MESH = Path(__file__).resolve().parent / "models" / "meshes" / "arm_slideway.stl"
_SOURCE_DIGEST = "b19c65c03a228077fc7466db2ae4fbc686a5eb7ffb36e8ec9806a8db6debe323"
_MESH_SCALE = "1.5 1 1"
_MESH_ORIGIN_XYZ = "0 0.08 0"
_COLLISION_ORIGIN_XYZ = "1.3680 0.0731 0.0515"
_COLLISION_BOX_SIZE = "3.2100 0.1838 0.1030"


def build_default_rail(*, member_id: str) -> SimpleNamespace:
    """构造默认单轴导轨视觉 mesh + 包围盒碰撞的只读 URDF 片段。"""

    resolved = _MESH.resolve()
    digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    if digest != _SOURCE_DIGEST:
        raise ValueError(f"default rail mesh digest drifted: {resolved.name}")
    root_link = f"{member_id}_base_link"
    uri = resolved.as_uri()
    mesh_xml = f"<geometry><mesh filename='{uri}' scale='{_MESH_SCALE}'/></geometry>"
    collision_xml = f"<geometry><box size='{_COLLISION_BOX_SIZE}'/></geometry>"
    visual_urdf = (
        f'<robot name="{member_id}_layout">'
        f'<link name="{root_link}">'
        f"<visual><origin xyz='{_MESH_ORIGIN_XYZ}' rpy='0 0 0'/>"
        f"{mesh_xml}"
        "<material name='rail_proxy'><color rgba='0.75 0.75 0.75 1'/></material>"
        "</visual>"
        f"<collision><origin xyz='{_COLLISION_ORIGIN_XYZ}' rpy='0 0 0'/>"
        f"{collision_xml}"
        "</collision>"
        "</link></robot>"
    )
    return SimpleNamespace(
        visual_urdf=visual_urdf,
        root_link=root_link,
        source_digest=_SOURCE_DIGEST,
        mesh_paths=(resolved,),
    )


__all__ = ["build_default_rail"]
