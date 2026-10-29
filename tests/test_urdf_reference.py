"""保留原参考模型检查，并以显式数值核对旋转后的棱柱运动。"""
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from unilab_robot_contracts.urdf_reference import Model, inertia_record
from unilab_end_effector_sim.urdf_reference import MimicModel


class ReferenceTests(unittest.TestCase):
    def test_impossible_inertia(self):
        x=ET.fromstring('<link><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="3" ixy="0" ixz="0" iyz="0"/></inertial></link>')
        self.assertFalse(inertia_record(x)['necessary_conditions_pass'])

    def test_planar_reference_fk(self):
        import math
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'a.urdf';p.write_text('<robot><link name="a"/><link name="b"/><link name="c"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint><joint name="fix" type="fixed"><parent link="b"/><child link="c"/><origin xyz="1 0 0"/></joint></robot>')
            fk=Model(p).fk([math.pi/2])['c'];self.assertAlmostEqual(fk[0][3],0);self.assertAlmostEqual(fk[1][3],1)

    def test_measured_mimic_is_not_replaced_by_command(self):
        xml='<robot><link name="root"/><link name="a"/><link name="b"/>'
        for name,parent,child,mimic in [('a','root','a',''),('b','a','b','<mimic joint="a" multiplier="-1"/>')]:
            xml+=f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/>{mimic}</joint>'
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'robot.urdf';p.write_text(xml+'</robot>');m=MimicModel(p)
            self.assertEqual(m.expand({'a':.3}),[.3,-.3])
            self.assertNotEqual(m.nominal_fk({'a':.3})['b'],m.fk([.3,-.1])['b'])
            for bad in ({'a':float('nan')},{'a':1.1},{'a':True},{'b':.3}):
                with self.assertRaises(ValueError):m.expand(bad)


def test_prismatic_motion_uses_rotated_joint_axis(tmp_path):
    """局部X先绕Z旋转90度，0.2米位移应沿父系Y。"""
    path = tmp_path / "model.urdf"
    path.write_text('<robot><link name="base"/><link name="tool"/>'
        '<joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>'
        '<origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="2 0 0"/>'
        '<limit lower="0" upper="1" velocity="1" effort="2"/></joint></robot>')
    pose = Model(path).fk([.2])["tool"]
    assert [pose[i][3] for i in range(3)] == pytest.approx([1., 2.2, 3.])


@pytest.mark.parametrize("positions", [[True], [float("nan")], [2.0], []])
def test_invalid_or_out_of_limit_observation_is_rejected(tmp_path, positions):
    """非法或越限观测不能通过参考正解的默认限位门禁。"""
    path = tmp_path / "model.urdf"
    path.write_text('<robot><link name="base"/><link name="tool"/>'
        '<joint name="joint" type="revolute"><parent link="base"/><child link="tool"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="2"/></joint></robot>')
    with pytest.raises(ValueError):
        Model(path).fk(positions)
