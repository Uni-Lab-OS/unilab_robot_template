"""Explicit deployment/model rail parameters; the legacy SZLab provider stays stable."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET

from .kinematic_model import RailKinematicModelBundle


@dataclass(frozen=True)
class RailConfiguration:
    model_id: str
    travel_m: tuple[float, float]
    velocity_limit_m_s: float
    direction: tuple[float, float, float]
    source: str

    def __post_init__(self):
        if not self.model_id or not self.source:
            raise ValueError("Rail configuration requires model identity and source")
        if len(self.travel_m) != 2 or len(self.direction) != 3:
            raise ValueError("Rail limits/axis dimensions invalid")
        values = (*self.travel_m, *self.direction, self.velocity_limit_m_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Rail parameters must be finite")
        if self.travel_m[0] >= self.travel_m[1] or self.velocity_limit_m_s <= 0:
            raise ValueError("Rail limits/speed invalid")
        if not math.isclose(sum(v*v for v in self.direction), 1., abs_tol=1e-9):
            raise ValueError("Rail direction must be a unit vector")

    @classmethod
    def from_mapping(cls, data):
        return cls(str(data['model_id']), tuple(map(float, data['travel_m'])),
                   float(data['velocity_limit_m_s']), tuple(map(float, data['direction'])),
                   str(data['source']))

    def as_mapping(self):
        return dict(model_id=self.model_id, travel_m=list(self.travel_m),
                    velocity_limit_m_s=self.velocity_limit_m_s,
                    direction=list(self.direction), source=self.source)

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self.as_mapping(), sort_keys=True,
                                        separators=(',', ':')).encode()).hexdigest()

    def contains(self, value):
        return math.isfinite(value) and self.travel_m[0] <= value <= self.travel_m[1]


def build_configured_model(*, device_id: str, configuration: RailConfiguration):
    """Build one prismatic axis; OS graph provides its installation transform."""
    if re.fullmatch(r'[A-Za-z0-9_]+', device_id) is None:
        raise ValueError('Invalid rail instance ID')
    robot = ET.Element('robot', name=device_id+'_rail')
    base, carriage, joint = (device_id+'_'+n for n in ('rail_base', 'rail_carriage', 'rail_joint'))
    for name in (base, carriage):
        ET.SubElement(robot, 'link', name=name)
    axis = ET.SubElement(robot, 'joint', name=joint, type='prismatic')
    ET.SubElement(axis, 'parent', link=base)
    ET.SubElement(axis, 'child', link=carriage)
    ET.SubElement(axis, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(axis, 'axis', xyz=' '.join(map(str, configuration.direction)))
    ET.SubElement(axis, 'limit', lower=str(configuration.travel_m[0]), upper=str(configuration.travel_m[1]),
                  velocity=str(configuration.velocity_limit_m_s), effort='0')
    semantic = ET.Element('robot', name=device_id+'_rail')
    group = ET.SubElement(semantic, 'group', name=device_id+'_axis')
    ET.SubElement(group, 'joint', name=joint)
    urdf = ET.tostring(robot, encoding='unicode')
    return RailKinematicModelBundle(render_urdf=urdf, source_digest=configuration.digest,
        qualified_joint_names=(joint,), topology_digest=hashlib.sha256(urdf.encode()).hexdigest(),
        mount_link=carriage, srdf=ET.tostring(semantic, encoding='unicode'))
