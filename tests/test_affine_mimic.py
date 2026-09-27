"""保留旧夹爪的独立仿射关系与错误依赖回归，不加载CTAG装配。"""
import unittest
import xml.etree.ElementTree as ET

from unilab_end_effector_sim.mimic import mimic_map


class MimicTests(unittest.TestCase):
    def test_nested_offsets_and_negative_scale(self):
        tree=ET.fromstring('<robot><joint name="a" type="revolute"/><joint name="b" type="revolute"><mimic joint="a" multiplier="-2" offset="0.1"/></joint><joint name="c" type="revolute"><mimic joint="b" multiplier="0.5" offset="0.3"/></joint></robot>')
        self.assertEqual(mimic_map(tree)['c'],('a',-1.,.35))

    def test_bad_dependency_rejected(self):
        for body in ('<mimic joint="a"/>','<mimic joint="missing"/>','<mimic joint="a" multiplier="nan"/>'):
            with self.assertRaises(ValueError):mimic_map(ET.fromstring('<robot><joint name="a" type="revolute">'+body+'</joint></robot>'))



def test_two_actuators_and_fixed_joint_do_not_merge():
    """独立驱动各自展开，固定关节不成为仿射输出。"""
    tree = ET.fromstring('<robot><joint name="mount" type="fixed"/>'
        '<joint name="left" type="revolute"/><joint name="right" type="prismatic"/>'
        '<joint name="follower" type="revolute"><mimic joint="left" multiplier="-0.5" offset="0.2"/></joint></robot>')
    assert mimic_map(tree) == {"left": ("left", 1.0, 0.0), "right": ("right", 1.0, 0.0),
                              "follower": ("left", -0.5, 0.2)}


def test_indirect_cycle_and_fixed_source_are_rejected():
    """间接循环和引用固定关节都不能变成可执行从动关系。"""
    for body in [
        '<joint name="a" type="revolute"><mimic joint="b"/></joint>'
        '<joint name="b" type="revolute"><mimic joint="a"/></joint>',
        '<joint name="fixed" type="fixed"/>'
        '<joint name="a" type="revolute"><mimic joint="fixed"/></joint>',
    ]:
        with unittest.TestCase().assertRaises(ValueError):
            mimic_map(ET.fromstring('<robot>' + body + '</robot>'))
