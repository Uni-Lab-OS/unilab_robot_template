"""Transaction regressions; real FCL overlap controls are recorded by the native qualification run."""
import copy
from types import SimpleNamespace
import pytest
from test_planning_scene_sync import planner,scene_request
from unilab_robot_runtime.world_body_collision import check_attached_bodies

def test_attached_probe_restores_binding_matrix_and_cache_after_query_error(planner):
    request=scene_request(attached=True,pairs=(("carrier","link_1"),))
    planner._load_scene(request)
    original=copy.deepcopy(planner.native)
    key=planner._loaded_scene_key
    call=planner._call
    def fail_validity(name,query):
        if name=="validity":
            attached=planner.native.robot_state.attached_collision_objects
            assert attached[0].link_name!="link_1"
            raise TimeoutError("injected validity failure")
        return call(name,query)
    planner._call=fail_validity
    with pytest.raises(TimeoutError,match="injected"):
        check_attached_bodies(planner,request)
    assert planner.native==original
    assert planner._loaded_scene_key==key

def test_unrecoverable_probe_restore_never_keeps_loaded_scene_cache(planner):
    request=scene_request(attached=True)
    planner._load_scene(request)
    call=planner._call
    queried=False
    def fail_restore(name,query):
        nonlocal queried
        if name=="validity":
            queried=True
            return SimpleNamespace(valid=True,contacts=[])
        if queried and query.scene.robot_state.attached_collision_objects:
            return SimpleNamespace(success=False)
        return call(name,query)
    planner._call=fail_restore
    with pytest.raises(RuntimeError,match="mutation rejected"):
        check_attached_bodies(planner,request)
    assert planner._loaded_scene_key is None

def test_external_native_attachments_cannot_share_the_probe_link(planner):
    import xml.etree.ElementTree as ET
    request=scene_request(attached=True)
    planner._load_scene(request)
    m=planner._session.messages
    for link in ET.fromstring(request.robot.urdf_xml).findall("link"):
        if link.find("collision") is not None:continue
        body=m["AttachedCollisionObject"]()
        body.link_name=link.get("name");body.object.id="external-"+body.link_name
        planner.native.robot_state.attached_collision_objects.append(body)
    original=copy.deepcopy(planner.native)
    with pytest.raises(ValueError,match="collision-free URDF link"):
        check_attached_bodies(planner,request)
    assert planner.native==original


@pytest.mark.parametrize("failed", ["robot", "world", "attachments"])
def test_complete_scene_gate_cannot_hide_any_collision(planner, monkeypatch, failed):
    from unilab_robot_runtime import world_body_collision as module
    from dataclasses import replace
    request = scene_request(attached=True)
    world = request.collision_world
    extra = replace(world.objects[0], object_id="released-material")
    request = replace(request, collision_world=replace(world, objects=(*world.objects, extra)))
    ids = ["released-material"]
    planner._state_validity = lambda *a, **kw: {"valid": failed != "robot"}
    monkeypatch.setattr(module, "check_world_bodies",
                        lambda *a: {"valid": failed != "world"})
    monkeypatch.setattr(module, "check_attached_bodies",
                        lambda *a: {"valid": failed != "attachments"})
    result = module.check_scene_state(planner, request, world_body_ids=ids)
    assert result["valid"] is False


def test_complete_scene_gate_uses_sampled_robot_state(planner, monkeypatch):
    from unilab_robot_runtime import world_body_collision as module
    from dataclasses import replace
    request = scene_request(attached=True)
    seen = []
    def validate(request, q, rail, **kwargs):
        seen.append((request.start_q, q, request.start_rail_m, rail))
        return {"valid": True}
    planner._state_validity = validate
    def attachments(planner, sampled):
        assert sampled.start_q == target
        return {"valid": True}
    target = tuple(v + .001 for v in request.start_q)
    monkeypatch.setattr(module, "check_attached_bodies", attachments)
    result = module.check_scene_state(planner, request, q=target)
    assert result["valid"]
    assert seen == [(target, target, request.start_rail_m, request.start_rail_m)]
    assert set(result["coverage"]) == {"robot_world", "moving_world", "moving_moving", "attachments"}
