"""任意设备包共用完整状态入口；缺位姿、未知关节或错模型均拒绝。"""
import numpy as np
import pytest
from unilab_robot_runtime.scene_request_builder import SceneRequestBuilder
from unilab_robot_runtime.cad_scene import CadScene,IDENTITY
from unilab_robot_contracts.planning import PlanGoal
from test_cad_scene import artifact
from test_planning_contract import _model

def test_state_required_and_payload_attach_release(tmp_path):
    cad=CadScene(artifact(tmp_path));robot=_model()
    builder=SceneRequestBuilder(cad,robot,None,lambda joints:{"link6":np.eye(4)},())
    state=dict(snapshot_id="fresh-1",source="os_kinematic_candidate",robot_model_digest=robot.digest,
               joint_positions=dict.fromkeys(robot.joint_names,0.),body_poses={"/body":IDENTITY},
               carried_body_ids=["/body"],held_material="material-A")
    request=builder.build(state,PlanGoal.from_joint([0.]*6))
    assert request.attachments[0].body_id=="/body"
    assert request.carried_body_ids==("/body",)
    released=builder.build(dict(state,carried_body_ids=[],held_material=None),request.goal)
    assert not released.attachments and released.collision_world.objects[0].attached_to_link is None
    with pytest.raises(ValueError,match="every moving"):
        builder.build(dict(state,body_poses={}),request.goal)
    with pytest.raises(ValueError,match="model"):
        builder.build(dict(state,robot_model_digest="wrong"),request.goal)
    with pytest.raises(ValueError,match="joint"):
        builder.build(dict(state,joint_positions={}),request.goal)


def test_authoritative_mount_survives_world_roundoff_and_rejects_stale_state(tmp_path):
    import copy, hashlib, json
    cad=CadScene(artifact(tmp_path));robot=_model()
    link=np.eye(4);link[0,3]=.37
    local=np.eye(4);local[0,3]=.12345678912345678
    world=link@local
    builder=SceneRequestBuilder(cad,robot,None,lambda joints:{"link6":link},())
    snapshot=dict(schema="unilab.attachment-state/v1",
        robot_urdf_sha256=hashlib.sha256(robot.urdf_xml.encode()).hexdigest(),
        bindings={"/body":dict(link_name="link6",link_from_body=local.tolist())})
    def seal(value):
        value.pop("revision",None)
        value["revision"]=hashlib.sha256(json.dumps(value,sort_keys=True,
            separators=(",",":"),allow_nan=False).encode()).hexdigest()
        return value
    state=dict(snapshot_id="current",source="os_kinematic_candidate",robot_model_digest=robot.digest,
        joint_positions=dict.fromkeys(robot.joint_names,0.),body_poses={"/body":world.tolist()},
        carried_body_ids=["/body"],held_material="material-A",attachment_state=seal(snapshot))
    goal=PlanGoal.from_joint([0.]*6)
    first=builder.build(state,goal)
    assert first.attachments[0].flange_from_body==tuple(map(tuple,local.tolist()))
    link[0,3]=.81
    state["body_poses"]["/body"]=(link@local).tolist()
    assert builder.build(state,goal).attachments==first.attachments
    assert first.options["scene_state"]["body_poses"]["/body"]==world.tolist()
    stale=copy.deepcopy(state);stale["body_poses"]["/body"][0][3]+=.001
    with pytest.raises(ValueError,match="projection"):builder.build(stale,goal)
    tampered=copy.deepcopy(state);tampered["attachment_state"]["bindings"]["/body"]["link_from_body"][0][3]+=.1
    with pytest.raises(ValueError,match="identity"):builder.build(tampered,goal)
    with pytest.raises(ValueError,match="inventory"):
        builder.build(dict(state,carried_body_ids=[],held_material=None),goal)
    wrong=copy.deepcopy(state);wrong["attachment_state"]["robot_urdf_sha256"]="other"
    seal(wrong["attachment_state"])
    with pytest.raises(ValueError,match="identity"):builder.build(wrong,goal)
    reflected=copy.deepcopy(state);reflected["attachment_state"]["bindings"]["/body"]["link_from_body"][0][0]=-1
    seal(reflected["attachment_state"])
    with pytest.raises(ValueError,match="rigid"):builder.build(reflected,goal)
