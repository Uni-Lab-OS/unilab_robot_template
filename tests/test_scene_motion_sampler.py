from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from unilab_robot_contracts.planning import CollisionWorld,CollisionObject,CollisionGeometry,AttachmentSpec
from unilab_robot_runtime.cad_scene import IDENTITY
from unilab_robot_runtime.scene_motion_sampler import SceneMotionSampler,interpolate_poses
from test_trajectory_artifact import fixture


def test_payload_follows_actual_saved_path_and_stays_after_release(tmp_path):
    req,result,artifact=fixture()
    state={"snapshot_id":"held","body_poses":{"carrier":IDENTITY},
           "carried_body_ids":["carrier"],"held_material":"carrier","rail_pose_origin_m":.03431498631834984,
           "attachment_state":{"revision":"fixture"}}
    world=CollisionWorld(objects=(CollisionObject("carrier",IDENTITY,
                (CollisionGeometry("box",size_m=(.01,.01,.01)),)),))
    req=replace(req,collision_world=world,
        attachments=(AttachmentSpec("carrier",req.robot.flange_link,IDENTITY),),
        options=dict(req.options,scene_state=state))
    artifact.append("carry",req,result)
    released=np.eye(4);released[0,3]=.1
    after=dict(state,snapshot_id="released",body_poses={"carrier":released.tolist()},
               carried_body_ids=[],held_material=None)
    before=dict(state,body_poses={"carrier":released.tolist()})
    artifact.events.append(dict(id="release",kind="release",time_s=1.,end_time_s=1.5,
        before=before,after=after))
    artifact.points.append(replace(artifact.points[-1],time_s=1.5))
    req2=replace(req,start_q=(.1,)*6,attachments=(),options=dict(req.options,scene_state=after))
    result2=replace(result,waypoints=tuple(replace(p,q=tuple(v+.1 for v in p.q))
                                           for p in result.waypoints),digest="")
    artifact.append("depart",req2,result2)
    path=tmp_path/"motion.json";artifact.save(path)
    def links(urdf,positions):
        matrix=np.eye(4);matrix[0,3]=positions[req.robot.joint_names[0]]
        return {req.robot.flange_link:matrix}
    sampler=SceneMotionSampler(path,req.robot,SimpleNamespace(axis_id="rail"),links)
    assert sampler.at(.6)["attachment_state"]=={"revision":"fixture"}
    assert "attachment_state" not in sampler.at(1.25)
    assert sampler.at(.6)["body_poses"]["carrier"][0][3]==pytest.approx(.06)
    assert sampler.at(2.1)["body_poses"]["carrier"][0][3]==pytest.approx(.1)
    assert sampler.at(2.1)["joint_positions"][req.robot.joint_names[0]]==pytest.approx(.16)
    assert sampler.at(1.5)["held_material"] is None
    assert sampler.at(2.1)["rail_pose_origin_m"]==.03431498631834984
    assert sampler.at(2.1)["source_artifact_digest"]==artifact.as_mapping()["digest"]


def test_event_rotation_interpolation_stays_rigid():
    from scipy.spatial.transform import Rotation
    a=np.eye(4);b=np.eye(4);b[:3,:3]=Rotation.from_euler("z",np.pi/2).as_matrix()
    middle=np.asarray(interpolate_poses({"body":a},{"body":b},.5)["body"])
    assert np.allclose(middle[:3,:3].T@middle[:3,:3],np.eye(3))
    assert Rotation.from_matrix(middle[:3,:3]).as_euler("xyz")[2]==pytest.approx(np.pi/4)


def playback_fixture(tmp_path):
    req,result,artifact=fixture()
    state=dict(body_poses={"carrier":IDENTITY},carried_body_ids=[],held_material=None)
    req=replace(req,options=dict(req.options,scene_state=state))
    artifact.append("move",req,result)
    path=tmp_path/"playback.json";artifact.save(path)
    sampler=SceneMotionSampler(path,req.robot,SimpleNamespace(axis_id="rail"),
                                lambda urdf,positions:{})
    return sampler


def test_playback_preserves_saved_points_and_checks_independent_readback(tmp_path):
    import copy
    sampler=playback_fixture(tmp_path)
    observed=copy.deepcopy(sampler.at(0.));calls=[]
    def apply(state):
        calls.append(state["time_s"])
        observed.clear();observed.update(copy.deepcopy(state))
    receipt=sampler.play(apply=apply,observe=lambda:observed,period_s=.35)
    assert receipt["status"]=="passed"
    assert set(p.time_s for p in sampler.points)<=set(calls)
    assert .35 in calls and calls[-1]==sampler.points[-1].time_s
    assert receipt["artifact_digest"]==sampler.artifact["digest"]


def test_playback_rejects_unplanned_bridge_before_any_write(tmp_path):
    sampler=playback_fixture(tmp_path);state=sampler.at(0.);calls=[]
    name=sampler.robot.joint_names[0];state["joint_positions"][name]+=.01
    with pytest.raises(ValueError,match="no unplanned bridge"):
        sampler.play(apply=calls.append,observe=lambda:state)
    assert calls==[]


def test_playback_stops_on_lost_readback_without_finishing(tmp_path):
    sampler=playback_fixture(tmp_path);state=sampler.at(0.);calls=[]
    def apply(sampled):
        calls.append(sampled["time_s"])
        # Simulate a stuck execution endpoint.
    with pytest.raises(ValueError,match="readback"):
        sampler.play(apply=apply,observe=lambda:state,period_s=.1)
    assert calls[-1]<sampler.points[-1].time_s
