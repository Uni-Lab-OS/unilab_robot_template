"""Measured trajectory quality; missing checks remain explicitly unqualified."""
import math
import numpy as np
from .urdf_kinematics import UrdfKinematics
from .trajectory_artifact import validate_timed_points,sample
from unilab_robot_contracts.planning import PlanWaypoint

def measure_segment(robot,rail_axis,segment,*,sample_period_s=.05):
    if "execution_curve" in segment:
        raise ValueError("Native-only metrics cannot describe a derived execution curve; use curve_joint_travel and its validation receipt")
    points=tuple(PlanWaypoint(**p) for p in segment["result"]["waypoints"])
    validate_timed_points(points);kin=UrdfKinematics(robot)
    times=sorted(set([p.time_s for p in points]+list(np.arange(0,points[-1].time_s,sample_period_s))))
    states=[sample(points,t) for t in times]
    poses=[kin.forward(p.q,{rail_axis.axis_id:p.rail_m}) for p in states]
    xyz=np.array([p[:3,3] for p in poses]);q=np.array([p.q for p in states])
    rail=np.array([p.rail_m for p in states])
    delta=xyz[-1]-xyz[0];length=float(np.linalg.norm(delta))
    if length>1e-10:
        fraction=np.clip((xyz-xyz[0])@delta/(length*length),0.,1.)
        line_error=np.linalg.norm(xyz-(xyz[0]+fraction[:,None]*delta),axis=1)
    else:line_error=np.linalg.norm(xyz-xyz[0],axis=1)
    carried=segment.get("scene_state",{}).get("carried_body_ids",[])
    payload_tilt=[]
    by_id={a["body_id"]:a for a in segment.get("attachments",[])}
    for identity in carried:
        relative=np.asarray(by_id[identity]["transform"])
        payload_tilt.extend(math.acos(float(np.clip((pose@relative)[2,2],-1.,1.))) for pose in poses)
    velocities=[abs(v) for p in points for v in (p.velocities or ())]
    accelerations=[abs(v) for p in points for v in (p.accelerations or ())]
    jerk=[]
    for a,b in zip(points,points[1:]):
        if a.accelerations is not None and b.accelerations is not None:
            jerk.extend(abs(y-x)/(b.time_s-a.time_s) for x,y in zip(a.accelerations,b.accelerations))
    return dict(id=segment["id"],duration_s=points[-1].time_s,raw_native_waypoints=len(points),
        measured_samples=len(states),arm_travel_rad=float(np.abs(np.diff(q,axis=0)).sum()),
        rail_travel_m=float(np.abs(np.diff(rail)).sum()),flange_path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
        flange_chord_m=length,max_flange_line_deviation_m=float(max(line_error)),
        max_payload_tilt_rad=max(payload_tilt,default=None),
        max_native_joint_velocity_rad_s=max(velocities,default=None),
        max_native_joint_acceleration_rad_s2=max(accelerations,default=None),
        max_native_sample_acceleration_slope_rad_s3=max(jerk,default=None),
        jerk_continuous_bound="not_qualified_for_piecewise_linear_position_interpolation",
        clearance_m=None,clearance_status="distance_not_available_from_state_validity_service",
        collision_status="native_dense_discrete_validation; not continuous proof")

def curve_joint_travel(curve):
    """Total variation of each saved polynomial coordinate, including reversals."""
    from scipy.interpolate import BSpline,PPoly
    knots=np.asarray(curve["knots"],dtype=float)
    coefficients=np.asarray(curve["coefficients"],dtype=float)
    degree=int(curve["degree"]);duration=float(curve["source_duration_s"])
    totals=[]
    for column in coefficients.T:
        spline=BSpline(knots,column,degree,extrapolate=False)
        derivative=PPoly.from_spline(spline).derivative()
        roots=derivative.roots(extrapolate=False)
        cuts=np.unique([0.,duration,*knots[(knots>0)&(knots<duration)],
                        *roots[np.isfinite(roots)&(roots>0)&(roots<duration)]])
        totals.append(float(np.abs(np.diff(spline(cuts))).sum()))
    return totals


import json, time
from pathlib import Path
from scipy.spatial.transform import Rotation
from unilab_robot_contracts.planning import PlanGoal, OrientationConstraintSpec
from .trajectory_artifact import read_artifact, _digest
from .trajectory_spline import validation_times, position_ranges
from .world_body_collision import check_scene_state
from .planning_scene_sync import apply_scene


def validate_smoothed_motion(provider,planner,path,output,*,linear_segments,
                             max_payload_tilt_rad,max_line_error_m,cartesian_velocity_limits,
                             restore_state=None):
    linear_speed,angular_speed=cartesian_velocity_limits
    if not all(math.isfinite(v) and v>0 for v in
               (max_payload_tilt_rad,max_line_error_m,linear_speed,angular_speed)):
        raise ValueError("Explicit positive process tolerances and Cartesian speed limits required")
    value,points=read_artifact(path)
    if value["schema"]!="unilab.timed-motion/v2":raise ValueError("Expected derived spline artifact")
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    kin=UrdfKinematics(provider.robot);axis=provider.rail_axis.axis_id
    report=dict(schema="unilab.spline-validation/v1",status="running",artifact_digest=value["digest"],
        scene_geometry_digest=provider.cad.digest,robot_model_digest=provider.robot.digest,
        sample_bounds=dict(joint_rad=.01,rail_m=.001,period_s=.1),
        continuous_collision_proof=False,hardware=False,segments=[],
        qualification_scope="motion_segments_only",scene_events="not_revalidated",
        minimum_clearance_m=None,clearance_qualification="distance_service_unavailable",
        contact_and_physics="not_run",limits_basis="explicit simulation engineering assumptions")
    if value["scene_geometry_digest"]!=provider.cad.digest:raise ValueError("Geometry identity differs")
    began=time.monotonic()
    def save():
        report["wall_s"]=time.monotonic()-began
        (output/"validation.json").write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf8")
    try:
        for segment in value["segments"]:
            snapshot=segment["scene_state"]
            q=tuple(snapshot["joint_positions"][n] for n in provider.robot.joint_names)
            rail=snapshot["joint_positions"][axis]
            orientation=tuple(OrientationConstraintSpec(**item) for item in
                segment["motion"]["constraints"]["orientation"])
            request=provider.request(snapshot,PlanGoal.from_joint(q),
                phase_contacts=tuple(tuple(pair) for pair in segment["allowed_collision_pairs"]),
                orientation=orientation)
            if request.collision_world.digest!=segment["scene_world_digest"]:
                raise ValueError("Reconstructed OS world differs from native source")
            curve=segment["execution_curve"]
            ranges=position_ranges(curve)
            limits=[*provider.robot.joint_limits,provider.rail_axis.limits_m]
            if any(lo<bound[0]-1e-9 or hi>bound[1]+1e-9 for (lo,hi),bound in zip(ranges,limits)):
                raise ValueError("Spline exceeded robot or rail position limits")
            for label,limit in (("velocity","velocity_limits"),("acceleration","acceleration_limits"),("jerk","jerk_limits")):
                if any(v>m*(1+1e-8) for v,m in zip(curve["maximum_"+label],curve[limit])):
                    raise ValueError("Spline "+label+" exceeded limit")
            times=validation_times(segment)
            first=sample(points,segment["start_time_s"]);last=sample(points,segment["end_time_s"])
            a=kin.forward(first.q,{axis:first.rail_m})[:3,3]
            b=kin.forward(last.q,{axis:last.rail_m})[:3,3]
            direction=b-a;length=float(direction@direction)
            row=dict(id=segment["id"],status="running",samples=0,total_samples=len(times),
                     max_line_error_m=0.,max_payload_tilt_rad=0.,position_ranges=ranges,
                     line_required=segment["id"] in linear_segments)
            report["segments"].append(row);save()
            held=provider.materials.get(snapshot.get("held_material"))
            attachment=next((a for a in segment["attachments"] if a["body_id"]==held),None)
            previous_pose=None;previous_time=None
            row["max_cartesian_speed_m_s"]=0.
            row["max_cartesian_angular_speed_rad_s"]=0.
            for index,local in enumerate(times):
                point=sample(points,segment["start_time_s"]+local)
                validity=check_scene_state(planner,request,q=point.q,rail_m=point.rail_m)
                row["samples"]+=1
                if not validity["valid"]:
                    row.update(status="failed",failure="collision_or_orientation",time_s=point.time_s,
                               state=point.as_mapping(),validity=validity)
                    raise ValueError("Derived curve state rejected in "+segment["id"])
                pose=kin.forward(point.q,{axis:point.rail_m})
                if row["line_required"] and previous_pose is not None and local-previous_time>1e-6:
                    dt=local-previous_time
                    speed=float(np.linalg.norm(pose[:3,3]-previous_pose[:3,3])/dt)
                    angular=float(Rotation.from_matrix(previous_pose[:3,:3].T@pose[:3,:3]).magnitude()/dt)
                    row["max_cartesian_speed_m_s"]=max(row["max_cartesian_speed_m_s"],speed)
                    row["max_cartesian_angular_speed_rad_s"]=max(row["max_cartesian_angular_speed_rad_s"],angular)
                    scale=float(segment["motion"]["velocity_scale"])
                    row["cartesian_speed_limit_m_s"]=linear_speed*scale
                    row["cartesian_angular_speed_limit_rad_s"]=angular_speed*scale
                    if speed>linear_speed*scale*(1+1e-3) or angular>angular_speed*scale*(1+1e-3):
                        raise ValueError("Derived contact LIN exceeded original Pilz Cartesian speed scale")
                previous_pose=pose;previous_time=local
                if row["line_required"]:
                    fraction=np.clip((pose[:3,3]-a)@direction/length,0.,1.) if length>1e-20 else 0.
                    error=float(np.linalg.norm(pose[:3,3]-(a+fraction*direction)))
                    row["max_line_error_m"]=max(row["max_line_error_m"],error)
                    if error>max_line_error_m:raise ValueError("Derived LIN error exceeded declared tolerance")
                if attachment:
                    body=pose@np.asarray(attachment["transform"])
                    tilt=math.acos(float(np.clip(body[2,2],-1.,1.)))
                    row["max_payload_tilt_rad"]=max(row["max_payload_tilt_rad"],tilt)
                    if tilt>max_payload_tilt_rad:raise ValueError("Derived payload tilt exceeded declared tolerance")
                if index%50==0:save()
            row["status"]="passed";save()
        report["status"]="passed_discrete_simulation_validation"
        value["qualification"]=dict(state=report["status"],hardware=False,
            scope="motion_segments_only",scene_events="not_revalidated",
            validation_artifact="validation.json",continuous_collision_proof=False,
            physical_qualification="not_run",minimum_clearance_qualification="not_run")
        value.pop("digest");value["digest"]=_digest(value)
        (output/"qualified-motion.json").write_text(json.dumps(value,indent=2,allow_nan=False),encoding="utf8")
        report["qualified_artifact_digest"]=value["digest"]
    except Exception as error:
        report["status"]="failed";report["error"]=repr(error)
        raise
    finally:
        save()
        if restore_state is not None:
            q=tuple(restore_state["joint_positions"][n] for n in provider.robot.joint_names)
            apply_scene(planner,provider.request(restore_state,PlanGoal.from_joint(q)))
    return report
