"""有限URDF树的CPU参考正解与惯量代数检查，不导入仿真器或驱动。"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .geometry import RigidTransform


def _matrix(transform: RigidTransform) -> list[list[float]]:
    """将共享刚体变换转换成旧参考模型的列向量矩阵输出。"""
    columns = [transform.rotate_vector(axis) for axis in ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))]
    return [[columns[j][i] for j in range(3)] + [transform.translation_m[i]] for i in range(3)] + [[0., 0., 0., 1.]]


def identity() -> list[list[float]]:
    return [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]


def mul(a, b):
    """组合已由模型生成的两个齐次矩阵；不作通用矩阵输入校验。"""
    return [[sum(float(a[i][k]) * float(b[k][j]) for k in range(4)) for j in range(4)] for i in range(4)]


def pose_to_matrix(xyz, quat_xyzw):
    return _matrix(RigidTransform(tuple(xyz), tuple(quat_xyzw)))


def rpy_to_quat_xyzw(rpy):
    return RigidTransform.from_rpy((0., 0., 0.), tuple(rpy)).orientation_xyzw


def numbers(value, n=3):
    v = [float(x) for x in value.split()]
    if len(v) != n or not all(math.isfinite(x) for x in v):
        raise ValueError('invalid numeric vector')
    return v

def origin(element):
    if element is None:
        return identity()
    return pose_to_matrix(numbers(element.get('xyz', '0 0 0')),
                          rpy_to_quat_xyzw(numbers(element.get('rpy', '0 0 0'))))

def axis_rotation(axis, angle):
    norm = math.sqrt(sum(x*x for x in axis))
    if norm < 1e-12:
        raise ValueError('zero joint axis')
    return pose_to_matrix([0, 0, 0], [*(x/norm*math.sin(angle/2) for x in axis), math.cos(angle/2)])

class Model:
    def __init__(self, path, allow_mimic=False):
        self.path = Path(path)
        self.xml = ET.parse(path).getroot()
        self.links = [x.get('name') for x in self.xml.findall('link')]
        if len(self.links) != len(set(self.links)):
            raise ValueError('duplicate link')
        self.joints = []
        for j in self.xml.findall('joint'):
            typ = j.get('type')
            if typ not in ('fixed', 'revolute', 'prismatic') or (j.find('mimic') is not None and not allow_mimic):
                raise ValueError('this preflight supports independent bounded joints only')
            a = j.find('axis'); lim = j.find('limit')
            d = dict(name=j.get('name'), type=typ, parent=j.find('parent').get('link'),
                     child=j.find('child').get('link'), origin=origin(j.find('origin')),
                     axis=numbers(a.get('xyz')) if a is not None else [1, 0, 0])
            if typ != 'fixed':
                d['limit'] = {k: float(lim.get(k)) for k in ('lower', 'upper', 'velocity', 'effort')}
                if not all(math.isfinite(v) for v in d['limit'].values()):
                    raise ValueError('nonfinite limit')
                if d['limit']['lower'] >= d['limit']['upper'] or min(d['limit']['velocity'], d['limit']['effort']) <= 0:
                    raise ValueError('invalid joint limits')
            self.joints.append(d)
        names = [j['name'] for j in self.joints]
        children = [j['child'] for j in self.joints]
        roots = set(self.links)-set(children)
        if len(names) != len(set(names)) or len(children) != len(set(children)) or len(roots) != 1:
            raise ValueError('URDF must be a tree')
        if any(j['parent'] not in self.links or j['child'] not in self.links for j in self.joints):
            raise ValueError('unknown link')
        self.root = next(iter(roots))
        self.moving = [j for j in self.joints if j['type'] != 'fixed']
        self.names = [j['name'] for j in self.moving]
        self.fk([0.0]*len(self.names), check_limits=False)

    def fk(self, q, check_limits=True):
        if len(q) != len(self.names) or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in q):
            raise ValueError('invalid joint positions')
        values = dict(zip(self.names, q))
        poses = {self.root: identity()}
        todo = list(self.joints)
        while todo:
            progressed = False
            for j in list(todo):
                if j['parent'] not in poses:
                    continue
                motion = identity()
                if j['type'] != 'fixed':
                    v = values[j['name']]
                    if check_limits and not j['limit']['lower'] <= v <= j['limit']['upper']:
                        raise ValueError('joint limit')
                    if j['type'] == 'revolute':
                        motion = axis_rotation(j['axis'], v)
                    else:
                        norm = math.sqrt(sum(x*x for x in j['axis']))
                        if norm == 0: raise ValueError('zero joint axis')
                        motion = pose_to_matrix([x/norm*v for x in j['axis']], [0,0,0,1])
                poses[j['child']] = mul(mul(poses[j['parent']], j['origin']), motion)
                todo.remove(j); progressed = True
            if not progressed:
                raise ValueError('cyclic or disconnected URDF')
        if set(poses) != set(self.links): raise ValueError('disconnected links')
        return poses

def inertia_record(link):
    """Numerical necessary conditions only; not physical identification."""
    item = link.find('inertial')
    if item is None: return None
    mass = float(item.find('mass').get('value'))
    raw = item.find('inertia').attrib
    v = {k: float(raw[k]) for k in ('ixx','ixy','ixz','iyy','iyz','izz')}
    a,b,c,d,e,f = [v[k] for k in ('ixx','ixy','ixz','iyy','iyz','izz')]
    def minors(a,b,c,d,e,f):
        return [a,d,f,a*d-b*b,a*f-c*c,d*f-e*e,a*d*f+2*b*c*e-a*e*e-d*c*c-f*b*b]
    eig_spd = all(x > 0 for x in minors(a,b,c,d,e,f))
    trace_half = (a+d+f)/2
    covariance_psd = all(x >= -1e-12 for x in minors(trace_half-a,-b,-c,trace_half-d,-e,trace_half-f))
    return {'mass_kg': mass, 'tensor': v, 'origin': item.find('origin').attrib if item.find('origin') is not None else {},
            'necessary_conditions_pass': math.isfinite(mass) and mass > 0 and all(math.isfinite(x) for x in v.values()) and eig_spd and covariance_psd,
            'measured_or_manufacturer_validated': False}
