"""Exact CAD collision snapshots shared by device packages and MoveIt backends.

Accepts the established body-local CAD export (the legacy postproc schema name
is supported as a wire alias). No station ids, custody rules or rail transforms
are inferred here. Static source-shape ids survive for contact attribution.
"""
from __future__ import annotations

import hashlib
import json
import zipfile
from dataclasses import replace
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from unilab_robot_contracts.planning import (
    AttachmentSpec, CollisionGeometry, CollisionObject, CollisionWorld,
)

IDENTITY = tuple(tuple(float(i == j) for j in range(4)) for i in range(4))


class _ArrayArchive:
    def __init__(self,archive):self.archive=archive
    def __getitem__(self,key):
        # NpzFile 的成员列表查询会对数万个小网格产生二次复杂度。
        # ZipFile 直接查中央目录，原始 NPY 数值与类型保持不变。
        with self.archive.open(key+".npy") as source:
            return np.lib.format.read_array(source,allow_pickle=False)


class _LazyObjects(Mapping):
    """Hash-verified immutable archive; materialize only requested object meshes."""
    def __init__(self, arrays, groups):
        self.arrays, self.groups, self.cache = arrays, groups, {}
        self.source_stat=(arrays.stat().st_size,arrays.stat().st_mtime_ns)
    def __iter__(self):return iter(self.groups)
    def __len__(self):return len(self.groups)
    def __getitem__(self, identity):
        if identity not in self.cache:
            if (self.arrays.stat().st_size,self.arrays.stat().st_mtime_ns)!=self.source_stat:
                raise ValueError('CAD array changed after digest verification')
            body, shapes = self.groups[identity]
            with zipfile.ZipFile(self.arrays) as archive:
                rows=[CadScene._shape(s,_ArrayArchive(archive)) for s in shapes]
            self.cache[identity]=CollisionObject(identity,body['world_from_body_initial'],
                tuple(g for g,_ in rows),role=body['role'],geometry_poses=tuple(p for _,p in rows))
        return self.cache[identity]


class CadScene:
    def __init__(self, geometry_file, *, expected_sha256=None, lazy=False):
        self.path = Path(geometry_file)
        raw = self.path.read_bytes()
        self.digest = hashlib.sha256(raw).hexdigest()
        if expected_sha256 is not None and self.digest != expected_sha256:
            raise ValueError("CAD metadata digest differs")
        self.metadata = json.loads(raw)
        data = self.metadata
        if data.get("status") != "exported" or data.get("unsupported"):
            raise ValueError("Incomplete CAD export")
        if data.get("units") != "metres" or data.get("matrix_convention") != "column_vectors":
            raise ValueError("CAD requires metres and column-vector transforms")
        arrays = self.path.parent / data["arrays"]
        if arrays.resolve().parent != self.path.parent.resolve():
            raise ValueError("CAD arrays must be a sibling artifact")
        if hashlib.sha256(arrays.read_bytes()).hexdigest() != data["arrays_sha256"]:
            raise ValueError("CAD array digest differs")
        self.bodies = {body["path"]: body for body in data["bodies"]}
        if len(self.bodies) != len(data["bodies"]):
            raise ValueError("Duplicate CAD body")
        self.object_bodies = {}
        declarations = {}
        for body in data['bodies']:
            groups = {}
            for shape in body['shapes']:
                identity = shape.get('collision_object_id')
                if identity is None:
                    identity=shape['prim'] if body['role']=='static_world' else body['path']
                if not isinstance(identity,str) or not identity:raise ValueError('Invalid collision object identity')
                groups.setdefault(identity,[]).append(shape)
            for identity,shapes in groups.items():
                if identity in declarations:raise ValueError('Duplicate CAD object: '+identity)
                declarations[identity]=(body,shapes);self.object_bodies[identity]=body['path']
        catalog=_LazyObjects(arrays,declarations)
        if lazy:self.objects=catalog
        else:
            self.objects={}
            with zipfile.ZipFile(arrays) as archive:
                indexed=_ArrayArchive(archive)
                for identity,(body,shapes) in declarations.items():
                    rows=[self._shape(shape,indexed) for shape in shapes]
                    self.objects[identity]=CollisionObject(identity,body['world_from_body_initial'],
                        tuple(g for g,_ in rows),role=body['role'],geometry_poses=tuple(p for _,p in rows))

    @staticmethod
    def _shape(shape, arrays):
        kind = shape["type"]
        if kind == "Mesh":
            if shape.get("point_frame") != "body_local":
                raise ValueError("Mesh points must already be body-local")
            vertices = arrays[shape["vertices"]]
            counts = arrays[shape["face_counts"]]
            indices = arrays[shape["face_indices"]]
            holes = set(shape.get("hole_faces", []))
            if sum(counts) != len(indices):
                raise ValueError("Invalid face topology")
            if np.any(counts != 3):
                raise ValueError("Nontriangular source collider")
            triangles=indices.reshape(-1,3)
            if holes:
                if min(holes)<0 or max(holes)>=len(counts):raise ValueError("Invalid hole face")
                mask=np.ones(len(counts),dtype=bool);mask[list(holes)]=False
                triangles=triangles[mask]
            from unilab_robot_contracts.mesh import PackedMeshRows
            return CollisionGeometry("mesh",
                vertices=PackedMeshRows(np.asarray(vertices,dtype="<f8").tobytes(),"d"),
                triangles=PackedMeshRows(np.asarray(triangles,dtype="<i8").tobytes(),"q")),IDENTITY
        matrix = np.array(shape["body_from_shape"], dtype=float)
        scale = np.linalg.norm(matrix[:3, :3], axis=0)
        if np.any(scale <= 0):
            raise ValueError("Invalid primitive scale")
        matrix[:3, :3] /= scale
        if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-7):
            raise ValueError("Sheared primitive requires exact mesh")
        params = shape["parameters"]
        if kind == "Cube":
            geometry = CollisionGeometry("box", size_m=tuple(scale * params["size"]))
        elif kind == "Cylinder":
            axis = {"X": 0, "Y": 1, "Z": 2}[params["axis"]]
            radial = [i for i in range(3) if i != axis]
            if abs(scale[radial[0]] - scale[radial[1]]) > 1e-9:
                raise ValueError("Elliptical cylinder requires exact mesh")
            alignment = np.eye(4)
            if axis == 0:
                alignment[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
            if axis == 1:
                alignment[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
            matrix = matrix @ alignment
            geometry = CollisionGeometry("cylinder", radius_m=params["radius"] * scale[radial[0]],
                                         length_m=params["height"] * scale[axis])
        else:
            raise ValueError("Unsupported exact CAD shape: " + kind)
        return geometry, tuple(map(tuple, matrix.tolist()))

    def snapshot(self, body_poses, attachments=(), *, flange_link, world_from_source=IDENTITY):
        """Require fresh poses for EVERY moving body; never use initial-pose fallback.

        Domain packages validate custody/contact policies before calling this.
        world_from_source is explicit; physical-world rail models pass identity.
        """
        moving = {key for key, body in self.bodies.items() if body["role"] != "static_world"}
        if set(body_poses) != moving:
            raise ValueError("Fresh poses must cover every moving CAD body exactly")
        attached = {a["body"]: a for a in attachments}
        if len(attached) != len(attachments) or set(attached) - moving:
            raise ValueError("Duplicate or unknown CAD attachment")
        specs = tuple(AttachmentSpec(a["body"], a.get("link_name",flange_link), a["flange_from_body"],
                                     tuple(a.get("touch_links", ())),a.get("purpose","tool_or_payload")) for a in attachments)
        transform = np.asarray(world_from_source, dtype=float)
        objects = []
        for key, original in self.objects.items():
            body_id = self.object_bodies[key]
            matrix = body_poses.get(body_id, original.world_pose)
            pose = tuple(map(tuple, (transform @ np.asarray(matrix)).tolist()))
            a = attached.get(body_id)
            objects.append(replace(original, world_pose=pose,
                                   attached_to_link=a.get("link_name",flange_link) if a else None,
                                   touch_links=tuple(a.get("touch_links", ())) if a else ()))
        return CollisionWorld(objects=tuple(objects)), specs
