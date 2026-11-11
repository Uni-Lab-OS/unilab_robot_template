"""通用 URDF 全树求值和有界候选逆解；不拥有设备工位、时钟或模型资产。

从 SZLab 已用实现提取，全树用于刚体帧投影；规划链仍可使用 UrdfKinematics。
外部轴独立于 active joint_names，不能作为机械臂规划自由度。
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

def vector(text, default=(0., 0., 0.)):
    return np.asarray(default if text is None else [float(x) for x in text.split()], dtype=float)

def origin_matrix(element):
    from scipy.spatial.transform import Rotation

    out = np.eye(4)
    if element is not None:
        out[:3, 3] = vector(element.get("xyz"))
        out[:3, :3] = Rotation.from_euler("xyz", vector(element.get("rpy"))).as_matrix()
    return out

class UrdfTreeKinematics:
    def __init__(self, urdf, joint_names, *, tip_frame="tool0", preferred_seed=None):
        self.path = Path(urdf)
        self.root = ET.parse(self.path).getroot()
        self.links = {x.get("name"): x for x in self.root.findall("link")}
        self.joints = {x.get("name"): x for x in self.root.findall("joint")}
        self.parent_joint = {x.find("child").get("link"): x for x in self.joints.values()}
        roots = set(self.links) - set(self.parent_joint)
        if len(roots) != 1:
            raise ValueError("URDF must have exactly one root")
        self.root_link = roots.pop()
        self.arm_names = list(joint_names)
        if not self.arm_names or len(set(self.arm_names)) != len(self.arm_names):
            raise ValueError("joint_names must be nonempty and unique")
        self.tip_frame = tip_frame
        self.preferred_seed = preferred_seed
        self.lower = np.array([float(self.joints[n].find("limit").get("lower")) for n in self.arm_names])
        self.upper = np.array([float(self.joints[n].find("limit").get("upper")) for n in self.arm_names])
        self.model_provider = None

    def frames(self, positions=None):
        from scipy.spatial.transform import Rotation

        positions = positions or {}
        if set(positions) - set(self.joints):
            raise ValueError("unknown joint positions")
        if not all(np.isfinite(float(v)) for v in positions.values()):
            raise ValueError("nonfinite joint positions")
        result = {self.root_link: np.eye(4)}
        remaining = list(self.joints.values())
        while remaining:
            advanced = False
            for joint in remaining[:]:
                parent = joint.find("parent").get("link")
                if parent not in result:
                    continue
                trans = origin_matrix(joint.find("origin"))
                kind = joint.get("type")
                axis_el = joint.find("axis")
                axis = vector(axis_el.get("xyz") if axis_el is not None else None, (1., 0., 0.))
                axis /= np.linalg.norm(axis) if np.linalg.norm(axis) else 1.
                q = float(positions.get(joint.get("name"), 0.))
                motion = np.eye(4)
                if kind in ("revolute", "continuous"):
                    motion[:3, :3] = Rotation.from_rotvec(axis*q).as_matrix()
                elif kind == "prismatic":
                    motion[:3, 3] = axis*q
                elif kind != "fixed":
                    raise ValueError("unsupported joint type: " + str(kind))
                result[joint.find("child").get("link")] = result[parent] @ trans @ motion
                remaining.remove(joint)
                advanced = True
            if not advanced:
                raise ValueError("URDF is not a connected tree")
        return result

    def fk(self, q, frame=None, external=None):
        values = np.asarray(q, dtype=float)
        if values.shape != (len(self.arm_names),) or not np.all(np.isfinite(values)):
            raise ValueError("invalid active joint positions")
        positions = dict(external or {})
        if set(positions) & set(self.arm_names):
            raise ValueError("external axes overlap active joint names")
        positions.update(zip(self.arm_names, values))
        return self.frames(positions)[frame or self.tip_frame]

    def ik(self, position, rotation, seed=None, external=None, frame=None, random_seed=0):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        goal = np.asarray(position, dtype=float)
        rot = np.asarray(rotation, dtype=float)
        initial = np.zeros(len(self.arm_names)) if seed is None else np.asarray(seed, dtype=float)
        rng = np.random.default_rng(random_seed)
        seeds = [initial]
        if self.preferred_seed is not None:
            seeds.append(np.asarray(self.preferred_seed, dtype=float))
        seeds += [rng.uniform(np.maximum(self.lower, -3.), np.minimum(self.upper, 3.)) for _ in range(16)]
        best = None
        def residual(q):
            pose = UrdfTreeKinematics.fk(self, q, frame, external)
            dr = Rotation.from_matrix(rot @ pose[:3, :3].T).as_rotvec()
            return np.r_[(pose[:3, 3]-goal)*4., dr]
        for start in seeds:
            fit = least_squares(residual, np.clip(start, self.lower+1e-6, self.upper-1e-6),
                                bounds=(self.lower+1e-7, self.upper-1e-7), max_nfev=450,
                                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            norm = np.linalg.norm(residual(fit.x))
            if best is None or norm < best[0]:
                best = (norm, fit.x)
            if norm < 1e-5:
                return fit.x
        raise ValueError(f"IK did not converge for {goal.tolist()}, weighted residual {best[0]:.6g}")


from functools import lru_cache

@lru_cache(maxsize=16)
def _moving_joint_metadata(urdf):
    result={}
    for joint in ET.fromstring(urdf).findall('joint'):
        if joint.get('type')=='fixed':continue
        limit=joint.find('limit');mimic=joint.find('mimic')
        result[joint.get('name')]=(None if limit is None else
            (float(limit.get('lower','-inf')),float(limit.get('upper','inf'))),
            None if mimic is None else (mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0'))))
    return result


def complete_joint_positions(urdf, active, passive=None):
    """补全显式非规划轴与 URDF mimic；拒绝缺失状态、覆盖活动轴及越界。"""
    metadata=_moving_joint_metadata(urdf);passive=dict(passive or {})
    if set(active)&set(passive):raise ValueError('passive joints overlap active joints')
    if set(passive)-set(metadata):raise ValueError('unknown passive joint')
    positions={**active,**passive}
    pending={name for name,(_,mimic) in metadata.items() if mimic is not None}
    while pending:
        ready=[name for name in pending if metadata[name][1][0] in positions]
        if not ready:break
        for name in ready:
            parent,scale,offset=metadata[name][1];value=float(positions[parent])*scale+offset
            if name in positions and abs(float(positions[name])-value)>1e-7:
                raise ValueError('mimic joint state differs from source relation')
            positions[name]=value;pending.remove(name)
    if set(metadata)-set(positions):raise ValueError('missing passive joint state: '+str(sorted(set(metadata)-set(positions))))
    for name,value in positions.items():
        value=float(value)
        if not np.isfinite(value):raise ValueError('nonfinite joint state')
        bounds=metadata.get(name,(None,None))[0]
        if bounds is not None and not bounds[0]<=value<=bounds[1]:raise ValueError('joint state outside URDF limits')
        positions[name]=value
    return positions
