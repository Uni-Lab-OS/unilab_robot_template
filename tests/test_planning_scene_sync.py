"""Native ROS message regressions with an in-memory scene service."""
import copy
from types import SimpleNamespace
import pytest
from unilab_robot_runtime.moveit_move_group import MoveGroupPlanner
from unilab_robot_runtime.planning_scene_sync import _set,_value
from unilab_robot_contracts.planning import (
    CollisionGeometry, CollisionObject,CollisionWorld,AttachmentSpec,PlanConstraints)
from test_move_group_planner_contract import request
from unilab_robot_runtime.cad_scene import IDENTITY

@pytest.fixture
def planner():
    p=MoveGroupPlanner()
    p._import_native()
    m=p._session.messages
    scene=m["PlanningScene"]()
    _set(scene.allowed_collision_matrix,"base_link","link_1",True,m["AllowedCollisionEntry"])
    p.native=scene
    p.reject=False
    def call(name,query):
        if p.reject:return SimpleNamespace(success=False)
        diff=query.scene
        if diff.name:p.native.name=diff.name
        if diff.robot_state.joint_state.name:
            p.native.robot_state.joint_state=copy.deepcopy(diff.robot_state.joint_state)
        if diff.allowed_collision_matrix.entry_names:
            p.native.allowed_collision_matrix=copy.deepcopy(diff.allowed_collision_matrix)
        assert diff.is_diff and diff.robot_state.is_diff
        world={obj.id:obj for obj in p.native.world.collision_objects}
        attached={obj.object.id:obj for obj in p.native.robot_state.attached_collision_objects}
        for obj in diff.robot_state.attached_collision_objects:
            if obj.object.operation==m["CollisionObject"].REMOVE:
                if obj.object.id not in attached:return SimpleNamespace(success=False)
                previous=attached.pop(obj.object.id)
                world[obj.object.id]=previous.object
                for other in list(p.native.allowed_collision_matrix.entry_names):
                    _set(p.native.allowed_collision_matrix,obj.object.id,other,False,m["AllowedCollisionEntry"])
            else:
                if not obj.object.meshes and not obj.object.primitives:
                    assert obj.object.id in world
                    obj=copy.deepcopy(obj)
                    obj.object=copy.deepcopy(world[obj.object.id])
                    obj.object.operation=m["CollisionObject"].ADD
                attached[obj.object.id]=obj;world.pop(obj.object.id,None)
        for obj in diff.world.collision_objects:
            if obj.operation==m["CollisionObject"].REMOVE:
                if obj.id not in world:return SimpleNamespace(success=False)
                world.pop(obj.id)
            elif obj.operation==m["CollisionObject"].MOVE:world[obj.id].pose=obj.pose
            else:world[obj.id]=obj
        p.native.world.collision_objects=list(world.values())
        p.native.robot_state.attached_collision_objects=list(attached.values())
        return SimpleNamespace(success=True)
    p._call=call
    p._read_scene=lambda **kwargs:copy.deepcopy(p.native)
    return p

def scene_request(attached=False, pairs=()):
    world=CollisionWorld(objects=(CollisionObject("carrier",IDENTITY,
        (CollisionGeometry("box",size_m=(.1,.1,.1)),)),))
    return request(collision_world=world,
        attachments=(AttachmentSpec("carrier","link_1",IDENTITY),) if attached else (),
        constraints=PlanConstraints(allowed_collision_pairs=pairs))

def test_grasp_release_and_phase_acm_restore_preserves_srdf(planner):
    first=scene_request(True,(("carrier","link_1"),))
    assert planner._load_scene(first)["attached"]==1
    assert _value(planner.native.allowed_collision_matrix,"carrier","link_1")
    result=planner._load_scene(scene_request())
    assert result["world_objects"]==1 and result["attached"]==0
    assert not _value(planner.native.allowed_collision_matrix,"carrier","link_1")
    assert _value(planner.native.allowed_collision_matrix,"base_link","link_1")

def test_rejected_apply_is_not_cached_and_can_retry(planner):
    planner.reject=True
    with pytest.raises(RuntimeError,match="rejected"):planner._load_scene(scene_request())
    assert planner._loaded_scene_key is None
    planner.reject=False
    assert planner._load_scene(scene_request())["verified"]

def test_world_identity_cache_retains_original_object(planner):
    a=scene_request().collision_world
    value=planner._world_digest(a)
    assert planner._world_digests[id(a)][0] is a
    assert value==a.digest


def test_attachment_mutations_do_not_erase_phase_matrix(planner):
    from dataclasses import replace
    first=scene_request(True,(("carrier","link_1"),))
    planner._load_scene(first)
    changed=replace(first,collision_world=CollisionWorld(objects=first.collision_world.objects+(
        CollisionObject("new-world",IDENTITY,(CollisionGeometry("box",size_m=(.01,.01,.01)),)),)))
    assert planner._load_scene(changed)["verified"]
    assert _value(planner.native.allowed_collision_matrix,"carrier","link_1")


def test_cached_geometry_still_updates_and_reads_back_robot_state(planner):
    from dataclasses import replace
    first=scene_request(True)
    planner._load_scene(first)
    changed=replace(first,start_q=(.1,)*first.robot.dof)
    result=planner._load_scene(changed)
    assert not result["reloaded"]
    assert result["robot_state"]["verified"]
    assert tuple(planner.native.robot_state.joint_state.position)==(.1,)*first.robot.dof
    assert [a.object.id for a in planner.native.robot_state.attached_collision_objects]==["carrier"]


def test_missing_joint_state_readback_is_rejected(planner):
    read=planner._read_scene
    def broken(**kwargs):
        value=read(**kwargs)
        value.robot_state.joint_state.position=[]
        return value
    planner._read_scene=broken
    with pytest.raises(RuntimeError,match="robot state readback"):
        planner._load_scene(scene_request())


def test_unchanged_attachment_survives_world_and_acm_updates_without_rebuild(planner):
    from dataclasses import replace
    first=scene_request(True, (("carrier","link_1"),))
    planner._load_scene(first)
    calls=[];original=planner._call
    def capture(name,query):
        calls.extend(query.scene.robot_state.attached_collision_objects)
        return original(name,query)
    planner._call=capture
    changed=replace(first,collision_world=CollisionWorld(objects=first.collision_world.objects+(
        CollisionObject("new-world",IDENTITY,(CollisionGeometry("box",size_m=(.01,.01,.01)),)),)),
        constraints=PlanConstraints())
    result=planner._load_scene(changed)
    assert result["retained_attachments"]==1
    assert calls==[]
    assert not _value(planner.native.allowed_collision_matrix,"carrier","link_1")
    assert len(planner.native.robot_state.attached_collision_objects)==1


def test_attachment_transform_and_geometry_changes_force_native_replacement(planner):
    from dataclasses import replace
    first=scene_request(True)
    planner._load_scene(first)
    moved=tuple(tuple(v+(.02 if i==0 and j==3 else 0) for j,v in enumerate(row)) for i,row in enumerate(IDENTITY))
    changed=replace(first,attachments=(replace(first.attachments[0],flange_from_body=moved),))
    assert planner._load_scene(changed)["retained_attachments"]==0
    assert planner.native.robot_state.attached_collision_objects[0].object.pose.position.x==.02
    changed=replace(changed,collision_world=CollisionWorld(objects=(
        replace(first.collision_world.objects[0],geometry=(CollisionGeometry("box",size_m=(.2,.1,.1)),)),)))
    assert planner._load_scene(changed)["retained_attachments"]==0
    assert planner.native.robot_state.attached_collision_objects[0].object.primitives[0].dimensions[0]==.2


def test_attached_world_projection_does_not_rebuild_native_local_geometry(planner):
    from dataclasses import replace
    first=scene_request(True)
    planner._load_scene(first)
    moved=tuple(tuple(v+(.2 if i==0 and j==3 else 0) for j,v in enumerate(row)) for i,row in enumerate(IDENTITY))
    changed=replace(first,start_q=(.1,),collision_world=CollisionWorld(objects=(
        replace(first.collision_world.objects[0],world_pose=moved),)))
    result=planner._load_scene(changed)
    assert result["equivalent_native_scene"] and not result["reloaded"]
    assert tuple(planner.native.robot_state.joint_state.position)==(.1,)
    assert planner.native.robot_state.attached_collision_objects[0].object.pose.position.x==0
    assert planner._loaded_scene_key==planner._scene_key(changed)
    # 世界中的独立物体同样移动时仍须下发 MOVE。
    released=replace(changed,attachments=())
    planner._load_scene(released)
    assert planner.native.world.collision_objects[0].pose.position.x==.2


def test_attachment_pose_update_reuses_existing_native_shapes(planner,monkeypatch):
    from dataclasses import replace
    import unilab_robot_runtime.moveit_move_group as backend
    first=scene_request(True)
    planner._load_scene(first)
    def forbidden(*args):
        raise AssertionError("unchanged mesh must not be serialized again")
    monkeypatch.setattr(backend,"_object_to_native",forbidden)
    moved=tuple(tuple(v+(.03 if i==1 and j==3 else 0) for j,v in enumerate(row)) for i,row in enumerate(IDENTITY))
    changed=replace(first,attachments=(replace(first.attachments[0],flange_from_body=moved),))
    result=planner._load_scene(changed)
    assert result["reused_attachment_geometry"]==1
    attached=planner.native.robot_state.attached_collision_objects[0]
    assert attached.object.pose.position.y==.03
    assert list(attached.object.primitives[0].dimensions)==[.1,.1,.1]
    assert not planner.native.world.collision_objects


def test_native_plan_start_inherits_attachments_only_after_verified_load(planner):
    req=scene_request(True)
    state=planner._state_with_scene_attachments(req,req.start_q,None)
    assert not state.is_diff and len(state.attached_collision_objects)==1
    planner._load_scene(req)
    state=planner._state_with_scene_attachments(req,req.start_q,None)
    assert state.is_diff and not state.attached_collision_objects
    planner._loaded_scene_key=None
    state=planner._state_with_scene_attachments(req,req.start_q,None)
    assert not state.is_diff and len(state.attached_collision_objects)==1


def test_restarted_native_scene_cannot_inherit_client_loaded_cache(planner):
    req=scene_request(True)
    first=planner._load_scene(req)
    assert first["native_scene_receipt"]==planner.native.name
    planner.native=planner._session.messages["PlanningScene"]()
    result=planner._load_scene(req)
    assert result["reloaded"] and result["native_scene_receipt"]!=first["native_scene_receipt"]
    assert len(planner.native.robot_state.attached_collision_objects)==1
    assert planner.native.name==planner._scene_receipt


def test_missing_native_acknowledgment_never_commits_cache(planner):
    original=planner._read_scene
    def missing(**kwargs):
        value=original(**kwargs)
        value.name=""
        return value
    planner._read_scene=missing
    with pytest.raises(RuntimeError,match="acknowledgment"):
        planner._load_scene(scene_request(True))
    assert planner._loaded_scene_key is None
    assert getattr(planner,"_scene_receipt",None) is None
