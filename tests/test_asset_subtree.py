"""几何分件不丢来源、不复用整件惯性，并拒绝不完整的选择声明。"""
import xml.etree.ElementTree as ET
import pytest
from unilab_robot_runtime.urdf_assets import asset_subtree


@pytest.fixture
def source(tmp_path):
    path=tmp_path/'robot.urdf'
    path.write_text('<robot name="split"><link name="bed"><inertial><mass value="10"/></inertial>'
        '<visual><geometry><box size="1 2 3"/></geometry></visual>'
        '<visual><geometry><box size="4 5 6"/></geometry></visual></link></robot>')
    return path


def test_geometry_subset_preserves_original_dimensions_without_duplicate_mass(source):
    part=asset_subtree(source,root_link='bed',prefix='plate_',descendants=False,
        collision_from_visual=True,root_visual_indices=(1,))
    link=ET.fromstring(part.render_urdf).find('link')
    assert link.find('visual/geometry/box').get('size')=='4 5 6'
    assert link.find('collision/geometry/box').get('size')=='4 5 6'
    assert link.find('inertial') is None
    assert part.sources['root_visual_indices']==[1]
    assert part.sources['files'][str(source)]==part.source_digest


@pytest.mark.parametrize('indices',[(),(2,),(0,0),(-1,),(True,)])
def test_invalid_subset_rejected(source,indices):
    with pytest.raises(ValueError):
        asset_subtree(source,root_link='bed',prefix='x_',collision_from_visual=True,root_visual_indices=indices)


def test_subset_cannot_keep_unfiltered_authored_collision(source):
    with pytest.raises(ValueError):asset_subtree(source,root_link='bed',prefix='x_',root_visual_indices=(1,))
