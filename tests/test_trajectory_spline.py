import numpy as np
import pytest
from unilab_robot_runtime.trajectory_spline import fit_curve,curve_spline,build_smoothed_artifact
from unilab_robot_runtime.trajectory_artifact import read_artifact,sample
from test_trajectory_artifact import fixture


def test_spline_preserves_knots_and_has_bounded_analytic_derivatives():
    req,result,artifact=fixture()
    limits=np.asarray([.05]*6+[.01])
    curve=fit_curve(result.waypoints,limits,limits,limits)
    spline=curve_spline(curve);scale=curve["time_scale"]
    for p in result.waypoints:assert np.allclose(spline(p.time_s),[*p.q,p.rail_m],atol=1e-12)
    for order in (1,2):
        assert np.max(np.abs(spline([0.,1.],nu=order)))<1e-10
    dense=np.linspace(0.,1.,10001)
    for order,key in ((1,"maximum_velocity"),(2,"maximum_acceleration"),(3,"maximum_jerk")):
        actual=np.max(np.abs(spline(dense,nu=order)),axis=0)/scale**order
        assert np.all(actual<=np.asarray(curve[key])+1e-8)
        assert np.all(np.asarray(curve[key])<=limits*(1+1e-8))


def test_shared_sampler_consumes_derived_curve_and_preserves_native_evidence(tmp_path):
    req,result,artifact=fixture();artifact.append("move",req,result)
    source=tmp_path/"native.json";target=tmp_path/"smooth.json"
    artifact.save(source)
    generated=build_smoothed_artifact(source,target,velocity=[.2]*7,acceleration=[.3]*7,jerk=[1.]*7)
    value,points=read_artifact(target)
    assert value["segments"][0]["result"]==result.as_mapping()
    assert value["qualification"]["state"]=="candidate_requires_revalidation"
    assert value["source_native_digest"]==artifact.as_mapping()["digest"]
    scale=value["segments"][0]["execution_curve"]["time_scale"]
    assert sample(points,.2*scale).q==pytest.approx((.02,)*6)
    assert sample(points,0.).velocities==pytest.approx((0.,)*6,abs=1e-12)
    assert sample(points,points[-1].time_s).accelerations==pytest.approx((0.,)*6,abs=1e-12)
    with pytest.raises(ValueError):sample(points,points[-1].time_s+.1)


def test_schema_cannot_silently_change_interpolation(tmp_path):
    import json
    from unilab_robot_runtime.trajectory_artifact import _digest
    _,result,artifact=fixture()
    req,_,_=fixture();artifact.append("move",req,result)
    value=artifact.as_mapping()
    value.pop("digest")
    value["interpolation"]="rest_boundary_quintic_bspline"
    value["digest"]=_digest(value)
    path=tmp_path/"mismatch.json";path.write_text(json.dumps(value),encoding="utf8")
    with pytest.raises(ValueError,match="interpolation"):read_artifact(path)


def test_selected_knots_retime_without_replacing_native_evidence(tmp_path):
    req,result,artifact=fixture();artifact.append("move",req,result)
    source=tmp_path/"native.json";target=tmp_path/"selected.json";artifact.save(source)
    chosen=[0,len(result.waypoints)-1]
    value=build_smoothed_artifact(source,target,velocity=[.2]*7,acceleration=[.3]*7,jerk=[1.]*7,
                                 knot_indices_by_segment={"move":chosen})
    _,points=read_artifact(target)
    assert value["segments"][0]["result"]==result.as_mapping()
    curve=value["segments"][0]["execution_curve"]
    assert curve["native_knot_indices"]==chosen
    assert sample(points,0.).q==pytest.approx(result.waypoints[0].q)
    assert sample(points,points[-1].time_s).q==pytest.approx(result.waypoints[-1].q)
    assert value["qualification"]["state"]=="candidate_requires_revalidation"


def test_stopped_edges_stay_on_certified_lines_and_bound_derivatives():
    from unilab_robot_contracts.planning import PlanWaypoint
    from unilab_robot_runtime.trajectory_spline import fit_stopped_edges
    points=[PlanWaypoint((0.,0.),0.,0.,"test"),PlanWaypoint((1.,0.),0.,1.,"test"),
            PlanWaypoint((1.,1.),0.,2.,"test")]
    curve=fit_stopped_edges(points,[.5]*3,[1.]*3,[3.]*3)
    spline=curve_spline(curve)
    middle=curve["stop_times_s"][1]
    before=spline(np.linspace(0,middle,1001));after=spline(np.linspace(middle,curve["duration_s"],1001))
    assert np.max(np.abs(before[:,1]))<1e-12
    assert np.max(np.abs(after[:,0]-1.))<1e-12
    for order in (1,2):assert np.max(np.abs(spline(middle,nu=order)))<1e-10
    assert max(curve["maximum_velocity"])<=.5*(1+1e-8)
    assert max(curve["maximum_acceleration"])<=1.*(1+1e-8)
    assert max(curve["maximum_jerk"])<=3.*(1+1e-8)

def test_curve_override_cannot_change_boundary_or_inherit_native_collision_verdict(tmp_path):
    req,result,artifact=fixture();artifact.append("move",req,result)
    source=tmp_path/"native.json";artifact.save(source)
    curve=fit_curve(result.waypoints,[.2]*7,[.3]*7,[1.]*7)
    value=build_smoothed_artifact(source,tmp_path/"derived.json",velocity=[.2]*7,
        acceleration=[.3]*7,jerk=[1.]*7,curve_overrides={"move":curve})
    assert value["qualification"]["state"]=="candidate_requires_revalidation"
    for row in curve["coefficients"]:row[0]+=.001
    with pytest.raises(ValueError,match="endpoint differs"):
        build_smoothed_artifact(source,tmp_path/"rejected.json",velocity=[.2]*7,
            acceleration=[.3]*7,jerk=[1.]*7,curve_overrides={"move":curve})
