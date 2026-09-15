"""CS66 六轴几何 URDF provider。"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

MESH_DIR = Path(__file__).with_name("meshes")
PARTS = ("base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3")
ORIGINS = (
    (0, 0, 0.1625, 0, 0, 0),
    (0, 0, 0, math.pi / 2, 0, 0),
    (-0.427, 0, 0, 0, 0, 0),
    (-0.3905, 0, 0.1475, 0, 0, 0),
    (0, -0.0965, 0, math.pi / 2, 0, 0),
    (0, 0.095, 0, -math.pi / 2, 0, 0),
)
HOME_DEG = (90.0, -65.0, -70.0, -135.0, 90.0, 0.0)


def source_digest() -> str:
    digest = hashlib.sha256(b"gripper-v4-cad-candidate-rail")
    digest.update(json.dumps(ORIGINS).encode())
    for part in PARTS:
        digest.update((MESH_DIR / (part + ".dae")).read_bytes())
    return digest.hexdigest()


SOURCE_DIGEST = source_digest()


def _visual(link, part):
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", filename=(MESH_DIR / (part + ".dae")).as_uri())


def _box_visual(link, size, rgba, xyz=(0, 0, 0)):
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz=" ".join(map(str, xyz)))
    ET.SubElement(ET.SubElement(visual, "geometry"), "box", size=" ".join(map(str, size)))
    ET.SubElement(ET.SubElement(visual, "material", name="process"), "color", rgba=rgba)


def build_base(*, member_id):
    root = ET.Element("robot", name=member_id)
    base = ET.SubElement(root, "link", name=member_id + "_static_base")
    geometry = ET.SubElement(ET.SubElement(base, "collision"), "geometry")
    ET.SubElement(geometry, "cylinder", radius=".075", length=".055")
    return SimpleNamespace(
        visual_urdf=ET.tostring(root, encoding="unicode"),
        root_link=member_id + "_static_base",
        source_digest=SOURCE_DIGEST,
        mesh_paths=(),
    )


def build_kinematics(
    *,
    device_id,
    position=None,
    rotation=None,
    base_xyz=None,
    rail_limits=None,
):
    del position, rotation
    if base_xyz is None or rail_limits is None:
        raise ValueError("build_kinematics 需要 base_xyz 与 rail_limits")
    root = ET.Element("robot", name=device_id)
    ET.SubElement(root, "link", name=device_id + "_base")
    names = tuple(f"{device_id}_joint_{i}" for i in range(1, 7))
    parent = device_id + "_carriage"
    _visual(ET.SubElement(root, "link", name=parent), "base")
    rail = ET.SubElement(root, "joint", name=device_id + "_rail_y", type="prismatic")
    ET.SubElement(rail, "parent", link=device_id + "_base")
    ET.SubElement(rail, "child", link=parent)
    ET.SubElement(rail, "axis", xyz="0 1 0")
    lo, hi = rail_limits
    base_y = float(base_xyz[1])
    ET.SubElement(
        rail,
        "limit",
        lower=str(lo - base_y),
        upper=str(hi - base_y),
        velocity="1",
        effort="1",
    )
    for i, (part, origin) in enumerate(zip(PARTS[1:], ORIGINS)):
        child = device_id + "_" + part
        _visual(ET.SubElement(root, "link", name=child), part)
        joint = ET.SubElement(root, "joint", name=names[i], type="revolute")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(map(str, origin[:3])),
            rpy=" ".join(map(str, origin[3:])),
        )
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(
            joint,
            "limit",
            lower=str(-2 * math.pi),
            upper=str(2 * math.pi),
            velocity="1",
            effort="1",
        )
        parent = child
    wrist = root.find("./link[@name='" + parent + "']")
    _box_visual(wrist, (0.08, 0.13, 0.025), ".22 .25 .28 1", (0, 0, 0.06))
    for side, sign in (("left", 1), ("right", -1)):
        name = device_id + "_jaw_" + side
        link = ET.SubElement(root, "link", name=name + "_link")
        _box_visual(link, (0.11, 0.008, 0.07), ".15 .75 .40 1", (0, 0, 0))
        joint = ET.SubElement(root, "joint", name=name, type="prismatic")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name + "_link")
        ET.SubElement(joint, "origin", xyz=f"0 {sign * 0.063} .105")
        ET.SubElement(joint, "axis", xyz=f"0 {-sign} 0")
        ET.SubElement(joint, "limit", lower="0", upper=".05", effort="1", velocity="1")
    names = names + (device_id + "_jaw_left", device_id + "_jaw_right", device_id + "_rail_y")
    digest = hashlib.sha256(
        json.dumps([SOURCE_DIGEST, device_id, names, ORIGINS]).encode()
    ).hexdigest()
    return SimpleNamespace(
        render_urdf=ET.tostring(root, encoding="unicode"),
        qualified_joint_names=names,
        topology_digest=digest,
        source_digest=SOURCE_DIGEST,
        mesh_paths=tuple(MESH_DIR / (part + ".dae") for part in PARTS),
        mount_link=parent,
    )
