import struct
import pytest
from unilab_robot_contracts.mesh import PackedMeshRows
from unilab_robot_contracts.planning import CollisionGeometry

def test_packed_mesh_is_immutable_and_keeps_exact_digest():
    vertices=((0.,0.,0.),(1.,0.,0.),(0.,1.,0.));triangles=((0,1,2),)
    v=PackedMeshRows(b"".join(struct.pack("<3d",*r) for r in vertices),"d")
    t=PackedMeshRows(struct.pack("<3q",0,1,2),"q")
    exact=CollisionGeometry("mesh",vertices=v,triangles=t)
    ordinary=CollisionGeometry("mesh",vertices=vertices,triangles=triangles)
    assert exact.as_mapping()==ordinary.as_mapping()
    import hashlib
    expected=b"[0.0,0.0,0.0][1.0,0.0,0.0][0.0,1.0,0.0][0,1,2]"
    assert exact.as_mapping()["mesh_sha256"]==hashlib.sha256(expected).hexdigest()
    assert list(exact.vertices)==list(vertices)
    assert exact.vertices is v
    with pytest.raises((TypeError,AttributeError)):v[0]=(1,2,3)
    with pytest.raises(ValueError,match="索引"):CollisionGeometry("mesh",vertices=v,
        triangles=PackedMeshRows(struct.pack("<3q",0,1,99),"q"))
