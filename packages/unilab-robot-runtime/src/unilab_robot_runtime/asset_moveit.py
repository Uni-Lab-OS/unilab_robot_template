"""Compile a locked USD serial mechanism to a cache-owned MoveIt model.

Geometry stays out of brand wheels. Joint frames, signed axes and collision
triangles come from the selected asset, not a similarly named vendor template.
No hardware or dynamics capability is granted by this kinematic adapter.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compile_asset_model(catalog, name, library_root, output, *, instance="robot", velocity=0.5):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pxr import Usd, UsdGeom, UsdPhysics

    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", instance):
        raise ValueError("Invalid instance namespace")
    if not math.isfinite(velocity) or velocity <= 0:
        raise ValueError("Explicit positive planning velocity required")
    root = catalog.asset(name, library_root)
    adapter = json.loads((root / "simready_usd/unilab.json").read_bytes())
    stage = Usd.Stage.Open(str(root / adapter["entrypoints"]["usd"]))
    if not stage or UsdGeom.GetStageUpAxis(stage) != "Z" or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-12:
        raise ValueError("Expected metre Z-up asset")
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use an empty output cache; no silent stale model reuse")
    output.mkdir(parents=True, exist_ok=True)
    cache = UsdGeom.XformCache()

    def world(prim):
        return np.array(cache.GetLocalToWorldTransform(prim), dtype=float).T

    def frame(pos, quat):
        m = np.eye(4)
        q = [*quat.GetImaginary(), quat.GetReal()]
        m[:3, :3] = Rotation.from_quat(q).as_matrix()
        m[:3, 3] = list(pos)
        return m

    joints = []; fixed_anchors=[]
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        p, c = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not p:  # A fixed world anchor is represented by the root mount below.
            if not prim.IsA(UsdPhysics.FixedJoint) or len(c)!=1 or not stage.GetPrimAtPath(c[0]):
                raise ValueError("Movable world joint unsupported")
            continue
        if len(p)==1 and len(c)==1 and prim.IsA(UsdPhysics.FixedJoint):
            fixed_anchors.append({'path':str(prim.GetPath()),'parent':str(p[0]),'child':str(c[0]),
                                  'a':frame(joint.GetLocalPos0Attr().Get(),joint.GetLocalRot0Attr().Get()),
                                  'b':frame(joint.GetLocalPos1Attr().Get(),joint.GetLocalRot1Attr().Get())})
            continue
        if len(p) != 1 or len(c) != 1 or not prim.IsA(UsdPhysics.RevoluteJoint):
            raise ValueError("This adapter requires an explicit revolute serial chain")
        if prim.GetAttribute("physics:excludeFromArticulation").Get():
            raise ValueError("Closed-loop/excluded joint requires a separate provider")
        a = frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
        b = frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
        rj = UsdPhysics.RevoluteJoint(prim)
        axis = str(rj.GetAxisAttr().Get())
        lower, upper = map(math.radians, (rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()))
        if axis not in ('X','Y','Z') or not all(map(math.isfinite, (lower, upper))) or lower >= upper:
            raise ValueError("Invalid source axis/domain")
        joints.append({"path": str(prim.GetPath()), "parent": str(p[0]), "child": str(c[0]), "a": a, "b": b, "axis": axis, "limits": (lower, upper)})
    roots = {j["parent"] for j in joints} - {j["child"] for j in joints}
    if len(roots) != 1 or len({j['child'] for j in joints}) != len(joints):
        raise ValueError("Not a uniquely rooted serial mechanism")
    base = next(iter(roots)); ordered = []; node = base
    anchor=base; consumed=[]
    while fixed_anchors:
        matches=[j for j in fixed_anchors if j['child']==anchor]
        if len(matches)!=1:raise ValueError('Non-root fixed joint requires a separate provider')
        j=matches[0];parent=stage.GetPrimAtPath(j['parent']);child=stage.GetPrimAtPath(j['child'])
        if j['parent'] in {v['parent'] for v in joints}|{v['child'] for v in joints} or any(p.IsA(UsdGeom.Boundable) for p in Usd.PrimRange(parent)):
            raise ValueError('Fixed ancestor has physical geometry; cannot discard it')
        if not np.allclose(world(parent)@j['a']@np.linalg.inv(j['b']),world(child),atol=2e-5):
            raise ValueError('Fixed root frames do not agree with authored USD')
        consumed.append({'source_joint':j['path'],'empty_parent_frame':j['parent'],'child':j['child'],'scope':'empty root frame folded into authored base_world mount'})
        anchor=j['parent'];fixed_anchors.remove(j)
    while len(ordered) < len(joints):
        children = [j for j in joints if j["parent"] == node]
        if len(children) != 1:
            raise ValueError("Branch or disconnected source joint")
        ordered.append(children[0]); node = children[0]["child"]
    if not 1 <= len(ordered) <= 7:
        raise ValueError("Unsupported independent joint count")
    bodies = [base] + [j["child"] for j in ordered]
    names = {p: f"{instance}_link_{i}" for i, p in enumerate(bodies)}
    urdf = ET.Element("robot", name=instance)
    ET.SubElement(urdf, "link", name="world")
    srdf = ET.Element("robot", name=instance)
    group = instance + "_arm"
    ET.SubElement(ET.SubElement(srdf, "group", name=group), "chain", base_link=names[base], tip_link=names[bodies[-1]])
    mesh_rows = []

    def origin(parent, m):
        if not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=2e-6) or not np.isclose(np.linalg.det(m[:3,:3]),1,atol=2e-6) or not np.allclose(m[3], [0, 0, 0, 1]):
            raise ValueError("Non-rigid body frame")
        ET.SubElement(parent, "origin", xyz=" ".join(format(x, ".16g") for x in m[:3, 3]), rpy=" ".join(format(x, ".16g") for x in Rotation.from_matrix(m[:3, :3]).as_euler("xyz")))

    for body in bodies:
        link = ET.SubElement(urdf, "link", name=names[body])
        bodyprim = stage.GetPrimAtPath(body)
        points = []; triangles = []
        for prim in Usd.PrimRange(bodyprim):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            # An internal nested rigid body must not be counted twice.
            if any(str(prim.GetPath()).startswith(other + "/") for other in bodies if other != body and other.startswith(body + "/")):
                continue
            mesh = UsdGeom.Mesh(prim)
            if UsdGeom.Imageable(prim).ComputeVisibility() == "invisible":
                continue
            vertices = np.array(mesh.GetPointsAttr().Get(), dtype=float)
            counts = list(mesh.GetFaceVertexCountsAttr().Get()); indices = list(mesh.GetFaceVertexIndicesAttr().Get())
            if not vertices.size or not np.isfinite(vertices).all() or any(n != 3 for n in counts) or len(indices)!=3*len(counts) or any(v<0 or v>=len(vertices) for v in indices):
                raise ValueError("Expected explicit complete source triangles")
            matrix = np.linalg.inv(world(bodyprim)) @ world(prim)
            vertices = (np.c_[vertices, np.ones(len(vertices))] @ matrix.T)[:, :3]
            offset = len(points); points.extend(vertices.tolist())
            triangles.extend((np.array(indices).reshape(-1, 3) + offset).tolist())
        if not triangles:
            raise ValueError("Missing collision geometry for source body: " + body)
        vertices = np.array(points); faces = np.array(triangles)
        path = output / (names[body] + ".stl")
        with path.open("wb") as stream:
            stream.write(b"USD exact surface triangles".ljust(80, b"\0")); stream.write(struct.pack("<I", len(faces)))
            tri = vertices[faces]
            normal = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
            lengths = np.linalg.norm(normal,axis=1)
            np.divide(normal,lengths[:,None],out=normal,where=lengths[:,None]!=0)
            records = np.zeros(len(faces),dtype=[('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            records['normal']=normal; records['vertices']=tri
            stream.write(records.tobytes())
        for role in ("visual", "collision"):
            geom = ET.SubElement(ET.SubElement(link, role), "geometry")
            ET.SubElement(geom, "mesh", filename=path.name)
        mesh_rows.append({"path": path.name, "sha256": sha(path), "source_body": body, "triangles": len(faces), "bounds": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]})
    mount = ET.SubElement(urdf, "joint", name=instance + "_mount", type="fixed")
    ET.SubElement(mount, "parent", link="world"); ET.SubElement(mount, "child", link=names[base]); origin(mount, world(stage.GetPrimAtPath(base)))
    q_home = []; joint_names = []; frame_rows = []
    for i, j in enumerate(ordered):
        pivot = f"{instance}_pivot_{i+1}"; joint_name = f"{instance}_joint_{i+1}"
        ET.SubElement(urdf, "link", name=pivot)
        node = ET.SubElement(urdf, "joint", name=joint_name, type="revolute")
        ET.SubElement(node, "parent", link=names[j["parent"]]); ET.SubElement(node, "child", link=pivot); origin(node, j["a"])
        axis = np.eye(3)["XYZ".index(j["axis"])]; ET.SubElement(node, "axis", xyz=" ".join(map(str, axis)))
        # Effort is an unused planning-only placeholder, never a hardware limit.
        ET.SubElement(node, "limit", lower=str(j["limits"][0]), upper=str(j["limits"][1]), velocity=str(velocity), effort="1")
        fixed = ET.SubElement(urdf, "joint", name=joint_name + "_body", type="fixed")
        ET.SubElement(fixed, "parent", link=pivot); ET.SubElement(fixed, "child", link=names[j["child"]]); origin(fixed, np.linalg.inv(j["b"]))
        ET.SubElement(srdf, "disable_collisions", link1=names[j["parent"]], link2=names[j["child"]], reason="Adjacent source joint")
        actual = np.linalg.inv(j["a"]) @ np.linalg.inv(world(stage.GetPrimAtPath(j["parent"]))) @ world(stage.GetPrimAtPath(j["child"])) @ j["b"]
        angle = float(Rotation.from_matrix(actual[:3, :3]).as_rotvec() @ axis)
        expected = np.eye(4); expected[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
        if not np.allclose(actual, expected, atol=2e-5):
            raise ValueError("Authored pose inconsistent with source joint frames: " + j["path"])
        q_home.append(angle); joint_names.append(joint_name)
        frame_rows.append({"source_joint": j["path"], "parent": names[j["parent"]], "child": names[j["child"]], "a": j["a"].tolist(), "b": j["b"].tolist(), "axis": axis.tolist()})
    (output / "robot.urdf").write_text(ET.tostring(urdf, encoding="unicode"), encoding="utf-8")
    (output / "robot.srdf").write_text(ET.tostring(srdf, encoding="unicode"), encoding="utf-8")
    parameters = adapter.get("parameters", {}).get("robot_parameters", {})
    model = {"schema": "unilab.asset-moveit/1", "model_id": catalog._read()["brand"] + "/" + name,
             "asset": catalog.model(name)["asset"], "instance": instance, "planning_group": group,
             "base_frame": names[base], "flange_link": names[bodies[-1]], "joint_names": joint_names,
             "joint_limits": [j["limits"] for j in ordered], "authored_q": q_home, "meshes": mesh_rows,
             "joint_frames": frame_rows, "base_world": world(stage.GetPrimAtPath(base)).tolist(),
             "controller_constraint": parameters.get("controller_constraint"),
             "controller_coordinate_matrix": parameters.get("controller_coordinate_matrix"),
             "kinematics": {group: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin", "kinematics_solver_timeout": 0.1}},
             "planning_assumptions": {"velocity_rad_s": velocity, "effort_placeholder": 1, "scope": "collision and kinematic planning only; no actuator dynamics or hardware execution"},
             "files": {"robot.urdf": sha(output / "robot.urdf"), "robot.srdf": sha(output / "robot.srdf")}}
    if consumed:model['fixed_root_frames']=consumed
    (output / "model.json").write_text(json.dumps(model, ensure_ascii=False, indent=2), encoding="utf-8")
    return {**model, 'model_file_sha256': sha(output / 'model.json')}


def load_compiled_model(folder, *, expected_sha256=None):
    """Load a portable generated cache, verifying XML and every collision mesh."""
    from unilab_robot_contracts.planning import RobotModel
    from unilab_robot_contracts._asset_package import inside
    folder = Path(folder).resolve()
    if expected_sha256 is not None and sha(folder / 'model.json') != expected_sha256:
        raise ValueError('Compiled model metadata differs from pinned build')
    data = json.loads((folder / "model.json").read_bytes())
    for name, expected in data["files"].items():
        if sha(inside(folder, name)) != expected:
            raise ValueError("Compiled model XML changed")
    xml = ET.fromstring((folder / "robot.urdf").read_text(encoding="utf-8"))
    hashes = {}
    for row in data["meshes"]:
        path = inside(folder, row["path"])
        if sha(path) != row["sha256"]:
            raise ValueError("Compiled collision geometry changed")
        hashes[path.as_uri()] = row["sha256"]
    for node in xml.findall(".//mesh"):
        node.set("filename", inside(folder, node.get("filename")).as_uri())
    robot = RobotModel(model_id=data["model_id"], urdf_xml=ET.tostring(xml, encoding="unicode"),
                       srdf_xml=(folder / "robot.srdf").read_text(encoding="utf-8"), base_frame=data["base_frame"],
                       flange_link=data["flange_link"], planning_group=data["planning_group"],
                       joint_names=tuple(data["joint_names"]), joint_limits=tuple(map(tuple, data["joint_limits"])),
                       kinematics=data["kinematics"], mesh_sha256=hashes)
    return robot, data


def source_coordinates(data, q):
    """Validate the original coupled controller domain in SI angular units."""
    import numpy as np
    values = np.asarray(q, dtype=float)
    if values.shape != (len(data['joint_names']),) or not np.isfinite(values).all():
        raise ValueError("Invalid model coordinates")
    limits = np.asarray(data['joint_limits'])
    if np.any(values < limits[:,0]-1e-10) or np.any(values > limits[:,1]+1e-10):
        raise ValueError('Coordinates violate asset joint limits')
    matrix = data.get("controller_coordinate_matrix")
    result = np.linalg.solve(np.asarray(matrix), values) if matrix else values
    domain = data.get("controller_constraint")
    if domain:
        lower = np.deg2rad(domain["source_lower"]); upper = np.deg2rad(domain["source_upper"])
        if np.any(result < lower - 1e-10) or np.any(result > upper + 1e-10):
            raise ValueError("Trajectory violates source coupled controller limits")
    return result.tolist()


class AssetMoveItPlanner:
    """Shared real MoveIt planner with pinned asset and coupled-domain checks.

    Coupled domains are checked at input and every returned waypoint. Their
    linear inverse maps make the straight interpolation domain convex. This
    post-filter may reject an OMPL result; it never claims native coupled-space
    search or silently returns an invalid trajectory as executable.
    """
    def __init__(self, folder, *, expected_sha256, config=None):
        from .moveit_planner import MoveItPlanner
        self.robot, self.data = load_compiled_model(folder, expected_sha256=expected_sha256)
        self.backend = MoveItPlanner(config) if config else MoveItPlanner()

    def plan(self, request):
        from dataclasses import replace
        from unilab_robot_contracts.planning import PlanResult
        if request.robot.digest != self.robot.digest:
            raise ValueError('Request model differs from pinned asset planning model')
        try:
            if request.goal.kind == 'pose' and request.goal.pose.frame_ref != 'world':
                raise ValueError('Asset pose goal must explicitly use world frame; transform other frames first')
            source_coordinates(self.data, request.start_q)
            if request.goal.kind == 'joint': source_coordinates(self.data, request.goal.joint_positions)
            if request.constraints.safe_pose: source_coordinates(self.data, request.constraints.safe_pose.joint_positions)
            for seed in request.seeds: source_coordinates(self.data, seed)
        except ValueError as exc:
            return PlanResult(status='invalid', waypoints=(), planner=self.backend.identity,
                              diagnostics={'failure_category':'source_coordinate_domain','reason':str(exc)}).with_payload(request)
        result = self.backend.plan(request)
        try:
            for waypoint in result.waypoints: source_coordinates(self.data, waypoint.q)
        except ValueError as exc:
            return replace(result, status='invalid', waypoints=(), digest='', diagnostics={**result.diagnostics,
                           'failure_category':'source_coordinate_domain', 'reason':str(exc)})
        return result

    def check_collision(self, request, q):
        if request.robot.digest != self.robot.digest:
            raise ValueError('Collision query model differs from pinned asset')
        source_coordinates(self.data, q)
        self.backend._ensure_robot(request)
        self.backend._load_world(request)
        return self.backend._collision_state(request, q)


def main():
    """Installed CLI; produces portable cache and prints the pin for planning."""
    import argparse
    import importlib
    p=argparse.ArgumentParser(description='Compile a pinned brand USD to an external MoveIt model cache')
    p.add_argument('--brand',required=True);p.add_argument('--model',required=True)
    p.add_argument('--library',required=True);p.add_argument('--output',required=True)
    p.add_argument('--instance',default='robot');a=p.parse_args()
    catalog=importlib.import_module('unilab_robot_'+a.brand).catalog
    result=catalog.compile_asset_model(a.model,a.library,a.output,instance=a.instance)
    print(json.dumps({'model':result['model_id'],'asset':result['asset'],'model_file_sha256':result['model_file_sha256'],'cache':str(Path(a.output).resolve())},ensure_ascii=False))


if __name__=='__main__':main()
