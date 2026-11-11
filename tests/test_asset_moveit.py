"""Integrity and coupled-coordinate rejection, independent of ROS installation."""
import json
import pytest
from unilab_robot_runtime.asset_moveit import load_compiled_model, source_coordinates
from unilab_robot_contracts.asset_cache import fetch_asset


def test_coupled_domain_is_not_axis_enclosure():
    data={'joint_names':['a','b'],'joint_limits':[[-4,4],[-4,4]],
          'controller_coordinate_matrix':[[1,0],[1,1]],
          'controller_constraint':{'source_lower':[-90,-90],'source_upper':[90,90]}}
    assert source_coordinates(data,[1,1.5])==[1,0.5]
    with pytest.raises(ValueError,match='coupled'): source_coordinates(data,[1,-1])
    with pytest.raises(ValueError,match='asset joint'): source_coordinates(data,[5,1])


def test_compiled_metadata_must_match_pin(tmp_path):
    (tmp_path/'model.json').write_text('{}')
    with pytest.raises(ValueError,match='metadata differs'): load_compiled_model(tmp_path,expected_sha256='0'*64)


def test_download_cannot_escape_cache(tmp_path):
    with pytest.raises(ValueError,match='Unsafe'):
        fetch_asset({'package_path':'../escape'},tmp_path,lambda _:pytest.fail('must reject before IO'))


def test_bad_download_never_becomes_asset(tmp_path):
    binding={'package_path':'robots/arm','asset_id':'test','package_digest':'0'*64}
    with pytest.raises(ValueError,match='hash/size'):
        fetch_asset(binding,tmp_path,lambda _:b'{}')
    assert not (tmp_path/'robots/arm').exists()
    assert len(list(tmp_path.glob('.asset-download-*/failure.json')))==1
