from dataclasses import replace
import hashlib
import pytest
from test_planning_contract import _model
from unilab_robot_runtime.urdf_assets import bind_mesh_identity


def model(path):
    return replace(_model(),urdf_xml='<robot name="test"><link name="base"><visual><geometry><mesh filename="'+path.as_posix()+'"/></geometry></visual></link></robot>')


def test_relocation_keeps_content_bound_identity(tmp_path):
    a=tmp_path/'a.stl';b=tmp_path/'another-install'/'b.stl';b.parent.mkdir()
    a.write_bytes(b'same source bytes');b.write_bytes(a.read_bytes())
    left=bind_mesh_identity(model(a),root=tmp_path)
    right=bind_mesh_identity(model(b),root=tmp_path)
    assert left.urdf_xml!=right.urdf_xml
    assert left.digest==right.digest
    assert 'sha256:' in left.identity_urdf_xml
    b.write_bytes(b'changed source bytes')
    with pytest.raises(ValueError,match='content differs'):bind_mesh_identity(right,root=tmp_path)
    assert bind_mesh_identity(model(b),root=tmp_path).digest!=left.digest


def test_partial_or_invalid_mesh_bindings_rejected(tmp_path):
    m=model(tmp_path/'a.stl')
    with pytest.raises(ValueError,match='cover every'):replace(m,mesh_sha256={'other':'a'*64})
    with pytest.raises(ValueError,match='SHA-256'):replace(m,mesh_sha256={(tmp_path/'a.stl').as_posix():'not-a-digest'})


def test_unbound_models_retain_legacy_identity():
    m=_model()
    assert m.as_mapping()['urdf_sha256']==hashlib.sha256(m.urdf_xml.encode()).hexdigest()
