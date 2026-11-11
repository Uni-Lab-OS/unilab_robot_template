"""Budgeted geometric candidate selection; output always requires curve revalidation."""
import math,time


def select_knots(points, edge_valid, *, max_edges=64, wall_s=60., clock=time.monotonic):
    if len(points)<2 or max_edges<1 or not math.isfinite(wall_s) or wall_s<=0:
        raise ValueError("Invalid candidate shortcut input or budget")
    deadline=clock()+wall_s;selected=[0];checks=[];exhausted=False
    while selected[-1]<len(points)-1:
        start=selected[-1];end=len(points)-1;accepted=None
        while end>start+1:
            if len(checks)>=max_edges or clock()>=deadline:
                exhausted=True;break
            try:
                valid=bool(edge_valid(points[start],points[end],deadline))
            except TimeoutError:
                exhausted=True;break
            checks.append(dict(start=start,end=end,valid=valid))
            if valid:accepted=end;break
            end=start+(end-start)//2
        if exhausted:
            selected.extend(range(start+1,len(points)));break
        selected.append(start+1 if accepted is None else accepted)
    return dict(indices=selected,checks=checks,budget_exhausted=exhausted,
                qualification="candidate_requires_curve_revalidation",
                original_adjacent_edges="retained native evidence")


import json
from pathlib import Path
from unilab_robot_contracts.planning import PlanGoal, PlanWaypoint, OrientationConstraintSpec
from .trajectory_artifact import read_artifact
from .trajectory_spline import build_smoothed_artifact
from .world_body_collision import check_scene_state
from .planning_scene_sync import apply_scene


def build_shortcut_candidate(provider,planner,source,output,*,segment_ids,restore_request,velocity,acceleration,jerk,stop_at_selected_knots=False,shortcut_wall_s=60.):
    value,_=read_artifact(source)
    if value["schema"]!="unilab.timed-motion/v1":raise ValueError("Original native artifact required")
    if value["scene_geometry_digest"]!=provider.cad.digest:raise ValueError("OS geometry differs")
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    selected={};rows=[];axis=provider.rail_axis.axis_id
    try:
        for segment in value["segments"]:
            if segment["id"] not in segment_ids:continue
            if segment.get("motion",{}).get("primitive")=="cartesian_linear":
                raise ValueError("Joint-space shortcut cannot replace required LIN")
            state=segment["scene_state"];q=tuple(state["joint_positions"][n] for n in provider.robot.joint_names)
            rail=state["joint_positions"][axis]
            orientation=tuple(OrientationConstraintSpec(**item) for item in
                segment["motion"]["constraints"]["orientation"])
            request=provider.request(state,PlanGoal.from_joint(q),orientation=orientation,
                phase_contacts=tuple(tuple(p) for p in segment["allowed_collision_pairs"]))
            if request.collision_world.digest!=segment["scene_world_digest"]:raise ValueError("OS world differs")
            apply_scene(planner,request)
            native=tuple(PlanWaypoint(**p) for p in segment["result"]["waypoints"])
            def edge(a,b,deadline):
                intervals=max(1,math.ceil(max(abs(x-y) for x,y in zip(a.q,b.q))/.01),
                              math.ceil(abs(a.rail_m-b.rail_m)/.001))
                for index in range(intervals+1):
                    if time.monotonic()>=deadline:raise TimeoutError("Shortcut query budget exhausted")
                    fraction=index/intervals
                    joints=tuple(x+(y-x)*fraction for x,y in zip(a.q,b.q))
                    axis_value=a.rail_m+(b.rail_m-a.rail_m)*fraction
                    if not check_scene_state(planner,request,q=joints,rail_m=axis_value)["valid"]:return False
                return True
            result=select_knots(native,edge,max_edges=24,wall_s=shortcut_wall_s)
            if not (stop_at_selected_knots and result["budget_exhausted"]):
                selected[segment["id"]]=result["indices"]
            result["applied_to_curve"]=segment["id"] in selected
            if not result["applied_to_curve"]:
                result["fallback"]="smooth full native path; do not stop at retained dense knots"
            rows.append(dict(id=segment["id"],native_points=len(native),**result))
            (output/"shortcut.json").write_text(json.dumps(rows,indent=2),encoding="utf8")
        candidate=build_smoothed_artifact(source,output/"candidate.json",
            velocity=velocity,acceleration=acceleration,jerk=jerk,
            knot_indices_by_segment=selected,stop_at_selected_knots=stop_at_selected_knots)
        return dict(status="candidate_requires_revalidation",duration_s=candidate["waypoints"][-1]["time_s"],
                    digest=candidate["digest"],segments=rows)
    finally:apply_scene(planner,restore_request)
