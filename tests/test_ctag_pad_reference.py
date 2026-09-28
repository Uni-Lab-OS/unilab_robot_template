"""CTAG指垫参考的原有穿越检查及固定网格读回检查。"""
import struct
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import pytest

from unilab_end_effector_sim.ctag_reference import pad_bounds, projected_pad_gap


class PadReferenceTests(unittest.TestCase):
    def test_pad_crossing_rejected_by_signed_gap(self):
        bounds={'ctag_Left_Pad_Link':[(0,0,0),(1,0,0)],'ctag_Right_Pad_Link':[(0,0,0),(1,0,0)]}
        pose=lambda x:[[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        self.assertEqual(projected_pad_gap(bounds,{'ctag_Left_Pad_Link':pose(2),'ctag_Right_Pad_Link':pose(0)}),1)
        self.assertLess(projected_pad_gap(bounds,{'ctag_Left_Pad_Link':pose(.5),'ctag_Right_Pad_Link':pose(0)}),0)


def _model(tmp_path, raw):
    (tmp_path / "pad.stl").write_bytes(raw)
    xml = ET.fromstring('<robot><link name="ctag_Left_Pad_Link"><visual><geometry>'
        '<mesh filename="pad.stl"/></geometry></visual></link>'
        '<link name="ctag_Right_Pad_Link"><visual><geometry>'
        '<mesh filename="pad.stl"/></geometry></visual></link></robot>')
    return SimpleNamespace(path=tmp_path / "model.urdf", xml=xml)


def _triangle():
    # 固定三角形三顶点的各轴极值不同，避免仅检查函数自身构造出的角点。
    points = (-2., 1., 4., 3., -5., 6., 1., 2., -7.)
    return b"CTAG reference".ljust(80, b" ") + struct.pack("<I12fH", 1, 0., 0., 1., *points, 0)


def test_binary_mesh_corners_are_read_from_vertices(tmp_path):
    bounds = pad_bounds(_model(tmp_path, _triangle()))
    expected = {(-2., -5., -7.), (-2., -5., 6.), (-2., 2., -7.), (-2., 2., 6.),
                (3., -5., -7.), (3., -5., 6.), (3., 2., -7.), (3., 2., 6.)}
    assert set(bounds) == {"ctag_Left_Pad_Link", "ctag_Right_Pad_Link"}
    assert all(set(corners) == expected for corners in bounds.values())


@pytest.mark.parametrize("tail", [b"extra", b""])
def test_binary_mesh_length_mismatch_is_rejected(tmp_path, tail):
    raw = _triangle() + tail if tail else _triangle()[:-1]
    with pytest.raises(ValueError, match="expected binary STL pads"):
        pad_bounds(_model(tmp_path, raw))


def test_gap_uses_palm_x_after_rotation():
    bounds = {"ctag_Left_Pad_Link": [(0., 0., 0.), (1., 2., 0.)],
              "ctag_Right_Pad_Link": [(0., 0., 0.), (1., 2., 0.)]}
    poses = {"ctag_Left_Pad_Link": [[0, -1, 0, 5], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
             "ctag_Right_Pad_Link": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}
    assert projected_pad_gap(bounds, poses) == 2.
