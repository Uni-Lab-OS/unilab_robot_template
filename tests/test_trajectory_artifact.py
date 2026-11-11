from dataclasses import replace
import json
import pytest
from unilab_robot_runtime.trajectory_artifact import TimedMotionArtifact,read_artifact,sample
from unilab_robot_contracts.planning import PlanRequest,PlanGoal,PlanResult,PlanWaypoint,PlannerIdentity
from test_planning_contract import _model

def fixture():
    robot=_model();geo="a"*64
    request=PlanRequest(robot=robot,start_q=(0.,)*6,goal=PlanGoal.from_joint([.1]*6),
        options={"scene_geometry_digest":geo,"scene_state":{"snapshot_id":"s1","source":"os_kinematic_candidate"}})
    points=(PlanWaypoint((0.,)*6,0.,0.,velocities=(0.,)*6,accelerations=(1.,)*6),
        PlanWaypoint((.02,)*6,0.,.2,velocities=(.2,)*6,accelerations=(1.,)*6),
        PlanWaypoint((.1,)*6,0.,1.,velocities=(0.,)*6,accelerations=(-1.,)*6))
    result=PlanResult(status="success",waypoints=points,planner=PlannerIdentity(name="test",implementation="test",version="1"))
    output=TimedMotionArtifact(robot_model_digest=robot.digest,scene_geometry_digest=geo,joint_names=robot.joint_names)
    return request,result,output

def test_roundtrip_retains_every_native_point_and_derivative(tmp_path):
    req,result,output=fixture();output.append("a",req,result)
    path=tmp_path/"motion.json";output.save(path);value,points=read_artifact(path)
    assert tuple(p.as_mapping() for p in points)==tuple(p.as_mapping() for p in result.waypoints)
    assert value["segments"][0]["result"]==result.as_mapping()
    # Demonstrates sampling the interior knot, not smoothstep between segment endpoints.
    assert sample(points,.2).q==(.02,)*6
    assert sample(points,.6).q==pytest.approx((.06,)*6)
    value["waypoints"][1]["q"][0]=.9;path.write_text(json.dumps(value))
    with pytest.raises(ValueError,match="digest"):read_artifact(path)

def test_reject_zero_time_failure_and_unplanned_bridge():
    req,result,output=fixture()
    bad=replace(result,waypoints=tuple(replace(p,time_s=0.) for p in result.waypoints),digest="")
    with pytest.raises(ValueError,match="strictly"):output.append("bad",req,bad)
    with pytest.raises(ValueError,match="failed"):output.append("bad",req,replace(result,status="collision",digest=""))
    output.append("first",req,result)
    with pytest.raises(ValueError,match="bridge"):output.append("reset",req,result)
    with pytest.raises(ValueError,match="identity"):output.append("wrong",replace(req,options=dict(req.options,scene_geometry_digest="b"*64)),result)

def test_continuous_join_has_no_invented_duration(tmp_path):
    req,result,output=fixture();output.append("first",req,result)
    next_req=replace(req,start_q=result.waypoints[-1].q,goal=PlanGoal.from_joint([.2]*6))
    next_result=replace(result,waypoints=tuple(replace(p,q=tuple(v+.1 for v in p.q)) for p in result.waypoints),digest="")
    output.append("second",next_req,next_result)
    assert len(output.points)==5
    assert output.points[-1].time_s==2.
    assert output.segments[1]["start_time_s"]==1.
