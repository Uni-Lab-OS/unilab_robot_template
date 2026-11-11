"""Lossless, scene-bound timed motion artifact shared by preview and candidate execution.

Interpolation is explicitly piecewise linear in joint/rail position, matching the
dense collision validator. Native derivatives are retained for audit; they do
not authorize a controller to substitute a different spline. Hardware execution
requires separate controller-interpolation validation.
"""
from bisect import bisect_right
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
from unilab_robot_contracts.planning import PlanWaypoint

SCHEMA = "unilab.timed-motion/v1"

def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),
        ensure_ascii=False,allow_nan=False).encode()).hexdigest()

def _same(a,b):
    return (len(a.q)==len(b.q) and max(abs(x-y) for x,y in zip(a.q,b.q))<=1e-6
            and abs(a.rail_m-b.rail_m)<=1e-7)

def validate_timed_points(points):
    if not points or abs(points[0].time_s)>1e-9:
        raise ValueError("Trajectory requires a zero-time initial state")
    if any(len(p.q)!=len(points[0].q) for p in points):
        raise ValueError("Trajectory joint dimensions differ")
    if len(points)==1:
        return
    if any(b.time_s<=a.time_s for a,b in zip(points,points[1:])):
        raise ValueError("Moving trajectory timestamps must strictly increase")

class TimedMotionArtifact:
    def __init__(self,*,robot_model_digest,scene_geometry_digest,joint_names):
        if any(len(d)!=64 or any(c not in "0123456789abcdef" for c in d)
               for d in (robot_model_digest,scene_geometry_digest)):
            raise ValueError("Model and geometry SHA-256 identities are required")
        self.model_digest=robot_model_digest;self.scene_digest=scene_geometry_digest
        self.joint_names=tuple(joint_names)
        if not self.joint_names or len(set(self.joint_names))!=len(self.joint_names):
            raise ValueError("Unique joint names required")
        self.segments=[];self.events=[];self.points=[];self.provenance={}

    @classmethod
    def load(cls,path):
        value,points=read_artifact(path)
        if value["schema"]!=SCHEMA:raise ValueError("Only native artifacts can resume native planning")
        result=cls(robot_model_digest=value["robot_model_digest"],
            scene_geometry_digest=value["scene_geometry_digest"],joint_names=value["joint_names"])
        result.segments=value["segments"];result.events=value["events"];result.points=list(points)
        result.provenance=value.get("provenance",{})
        return result

    def append(self,segment_id,request,result):
        if result.status!="success":raise ValueError("Cannot export failed motion as executable")
        if request.robot.digest!=self.model_digest:raise ValueError("Robot identity changed")
        if segment_id in {s["id"] for s in self.segments}:raise ValueError("Duplicate segment ID")
        snapshot=request.options.get("scene_state")
        if not snapshot:raise ValueError("A current OS scene-state provenance is required")
        if request.options.get("scene_geometry_digest")!=self.scene_digest:
            raise ValueError("Scene geometry identity changed")
        points=tuple(result.waypoints);validate_timed_points(points)
        if len(points[0].q)!=len(self.joint_names):raise ValueError("Joint dimension mismatch")
        start=PlanWaypoint(tuple(request.start_q),float(request.start_rail_m or 0.),0.)
        if not _same(start,points[0]):raise ValueError("Planner trajectory does not start at request state")
        if self.points and not _same(self.points[-1],points[0]):
            raise ValueError("Unplanned inter-segment bridge rejected")
        offset=self.points[-1].time_s if self.points else 0.
        shifted=[replace(p,time_s=offset+p.time_s) for p in points]
        self.points.extend(shifted[1:] if self.points else shifted)
        self.segments.append(dict(id=segment_id,start_time_s=offset,end_time_s=offset+points[-1].time_s,
            request_digest=request.digest,scene_world_digest=request.collision_world.digest,
            motion=dict(primitive=request.options.get("primitive"),
                pipeline=request.options.get("pipeline"),planner_id=request.options.get("planner_id"),
                goal=request.goal.as_mapping(),
                cartesian_intent=request.options.get("cartesian_intent"),
                constraints=request.constraints.as_mapping(),
                velocity_scale=request.options.get("velocity_scale"),
                acceleration_scale=request.options.get("acceleration_scale")),
            scene_state=snapshot,attachments=[dict(body_id=a.body_id,link_name=a.link_name,
                transform=[list(row) for row in a.flange_from_body],touch_links=list(a.touch_links),purpose=a.purpose)
                for a in request.attachments],
            allowed_collision_pairs=[list(p) for p in request.constraints.allowed_collision_pairs],
            result=result.as_mapping()))

    def event(self,event_id,kind,details):
        if not self.points:raise ValueError("Event requires a known candidate state")
        self.events.append(dict(id=event_id,kind=kind,time_s=self.points[-1].time_s,details=details))

    def as_mapping(self):
        if not self.segments:raise ValueError("Empty motion artifact")
        value=dict(schema=SCHEMA,robot_model_digest=self.model_digest,
            scene_geometry_digest=self.scene_digest,joint_names=list(self.joint_names),
            interpolation="piecewise_linear_joint_and_rail",hardware=False,
            segments=self.segments,events=self.events,
            waypoints=[p.as_mapping() for p in self.points])
        if getattr(self,"provenance",None):value["provenance"]=self.provenance
        value["digest"]=_digest(value)
        return value

    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        tmp=path.with_suffix(path.suffix+".tmp")
        tmp.write_text(json.dumps(self.as_mapping(),ensure_ascii=False,indent=2,allow_nan=False),encoding="utf8")
        tmp.replace(path)

def read_artifact(path):
    value=json.loads(Path(path).read_text(encoding="utf8"))
    claimed=value.pop("digest",None)
    if value.get("schema") not in (SCHEMA,"unilab.timed-motion/v2") or claimed!=_digest(value):
        raise ValueError("Motion artifact schema/digest differs")
    expected_interpolation={SCHEMA:"piecewise_linear_joint_and_rail",
                            "unilab.timed-motion/v2":"rest_boundary_quintic_bspline"}
    if value.get("interpolation") != expected_interpolation[value["schema"]]:
        raise ValueError("Unsupported trajectory interpolation")
    points=tuple(PlanWaypoint(**p) for p in value["waypoints"])
    validate_timed_points(points)
    value["digest"]=claimed
    if value["schema"]=="unilab.timed-motion/v2":
        from .trajectory_spline import CurvePoints,INTERPOLATION
        if value["interpolation"]!=INTERPOLATION:raise ValueError("Curve artifact interpolation differs")
        points=CurvePoints(points,value["segments"])
    return value,points

def sample(points,time_s):
    """Sample the saved path only. No endpoint bridging, easing, or speed override."""
    if hasattr(points,"sample"):return points.sample(time_s)
    if not math.isfinite(time_s) or time_s<0 or time_s>points[-1].time_s+1e-9:
        raise ValueError("Sample time is outside artifact")
    index=min(bisect_right([p.time_s for p in points],time_s)-1,len(points)-1)
    left=points[index]
    if index==len(points)-1:return replace(left,time_s=time_s)
    right=points[index+1];alpha=(time_s-left.time_s)/(right.time_s-left.time_s)
    return PlanWaypoint(tuple(a+(b-a)*alpha for a,b in zip(left.q,right.q)),
        left.rail_m+(right.rail_m-left.rail_m)*alpha,time_s,"artifact_interpolation")
