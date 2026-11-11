from types import SimpleNamespace
import numpy as np
import pytest
from unilab_robot_contracts.planning import CollisionGeometry,CollisionObject
from unilab_robot_runtime.collision_instance import CollisionInstance
from unilab_robot_runtime.fcl_collision import FCLSceneChecker

def test_two_instances_share_geometry_and_have_independent_world_poses():
    initial=np.eye(4);initial[0,3]=.4
    original=CollisionObject('/axis',tuple(map(tuple,initial)),(CollisionGeometry('box',size_m=(.1,.1,.1)),))
    catalog=SimpleNamespace(objects={'/axis':original},object_bodies={'/axis':'/axis'})
    placement=np.eye(4);placement[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];placement[1,3]=2.
    one=CollisionInstance(catalog,'os-instance-1',placement)
    other=CollisionInstance(catalog,'os-instance-2',np.eye(4))
    a=one['os-instance-1::/axis'];b=other['os-instance-2::/axis']
    assert a.geometry is b.geometry
    assert np.allclose(np.array(a.world_pose)[:3,3],[0,2.4,0])
    assert b.world_pose==original.world_pose
    checker=FCLSceneChecker([a,b])
    assert not checker.check({a.object_id:a.world_pose},moving_ids=[a.object_id])['collision']
    assert checker.check({a.object_id:b.world_pose},moving_ids=[a.object_id])['collision']
    assert b.world_pose==original.world_pose

def test_invalid_instance_scale_is_rejected():
    catalog=SimpleNamespace(objects={},object_bodies={});scaled=np.eye(4);scaled[0,0]=2
    with pytest.raises(ValueError,match='Rigid'):CollisionInstance(catalog,'x',scaled)
