"""Exact scene input, per-shape frames, stale state and tampering regression."""
import hashlib,json
import numpy as np
import pytest
from unilab_robot_runtime.cad_scene import CadScene, IDENTITY
from unilab_robot_contracts.planning import CollisionGeometry,CollisionObject

def artifact(tmp_path):
    np.savez(tmp_path/"geometry.npz",v=np.array([[0.,0,0],[1,0,0],[0,1,0]]),
             n=np.array([3]),i=np.array([0,1,2]))
    matrix=np.eye(4);matrix[0,3]=.3
    document=dict(status="exported",units="metres",matrix_convention="column_vectors",
        arrays="geometry.npz",arrays_sha256=hashlib.sha256((tmp_path/"geometry.npz").read_bytes()).hexdigest(),
        bodies=[dict(path="/body",role="material",world_from_body_initial=np.eye(4).tolist(),
            shapes=[dict(type="Mesh",point_frame="body_local",vertices="v",face_counts="n",
                         face_indices="i",hole_faces=[],prim="/body/mesh"),
                    dict(type="Cube",body_from_shape=matrix.tolist(),parameters={"size":.1},prim="/body/cube")])])
    (tmp_path/"geometry.json").write_text(json.dumps(document))
    return tmp_path/"geometry.json"

def test_shape_frame_is_not_applied_to_body_local_mesh_twice(tmp_path):
    scene=CadScene(artifact(tmp_path))
    obj=scene.objects["/body"]
    assert obj.geometry[0].vertices[1]==(1,0,0)
    assert obj.geometry_poses[0]==IDENTITY
    assert obj.geometry_poses[1][0][3]==.3
    assert obj.geometry[1].size_m==(.1,.1,.1)

def test_missing_dynamic_pose_is_rejected_and_attachment_is_explicit(tmp_path):
    scene=CadScene(artifact(tmp_path))
    with pytest.raises(ValueError,match="every moving"):
        scene.snapshot({},flange_link="flange")
    pose=np.eye(4);pose[2,3]=.5
    world,attached=scene.snapshot({"/body":pose},[dict(body="/body",flange_from_body=IDENTITY,touch_links=[])],
                                flange_link="flange")
    assert world.objects[0].world_pose[2][3]==.5
    assert attached[0].link_name=="flange"
    released,_=scene.snapshot({"/body":pose},flange_link="flange")
    assert released.objects[0].attached_to_link is None

def test_array_tamper_fails_closed(tmp_path):
    path=artifact(tmp_path);(tmp_path/"geometry.npz").write_bytes(b"tamper")
    with pytest.raises(ValueError,match="digest"):CadScene(path)

def test_geometry_pose_affects_identity_and_cache_preserves_digest():
    shape=CollisionGeometry("box",size_m=(.1,.1,.1))
    original=CollisionObject("box",IDENTITY,(shape,))
    moved=np.eye(4);moved[0,3]=.3
    altered=CollisionObject("box",IDENTITY,(shape,),geometry_poses=(tuple(map(tuple,moved)),))
    assert original.as_mapping()!=altered.as_mapping()


def test_explicit_groups_and_lazy_materialization_match_eager(tmp_path):
    path=artifact(tmp_path);data=json.loads(path.read_text())
    shapes=data['bodies'][0]['shapes']
    shapes[0]['collision_object_id']='/body/contact'
    shapes[1]['collision_object_id']='/body/structure'
    path.write_text(json.dumps(data))
    lazy=CadScene(path,lazy=True);eager=CadScene(path)
    assert set(lazy.objects)=={'/body/contact','/body/structure'}
    assert len(lazy.objects.cache)==0
    assert lazy.objects['/body/contact']==eager.objects['/body/contact']
    assert len(lazy.objects.cache)==1
    assert lazy.object_bodies==eager.object_bodies
    assert lazy.objects['/body/structure']==eager.objects['/body/structure']

def test_lazy_source_change_fails_closed(tmp_path):
    path=artifact(tmp_path);scene=CadScene(path,lazy=True)
    (tmp_path/'geometry.npz').write_bytes(b'changed after lazy catalog creation')
    with pytest.raises(ValueError,match='changed'):scene.objects['/body']
