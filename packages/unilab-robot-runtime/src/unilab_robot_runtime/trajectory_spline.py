"""保留全部几何路点的 C2 以上样条与显式速度、加速度、jerk 时间缩放。

新曲线必须重新做碰撞、LIN 和姿态检查；本模块不授予执行资格。
"""
import copy,json,math
from pathlib import Path
from bisect import bisect_right
from dataclasses import replace
import numpy as np
from scipy.interpolate import BSpline,PPoly,make_interp_spline
from unilab_robot_contracts.planning import PlanWaypoint

SCHEMA="unilab.timed-motion/v2"
INTERPOLATION="rest_boundary_quintic_bspline"


def extrema(spline,order):
    """解析每段多项式的极值，包含全部内部导数零点。"""
    result=[]
    for axis in range(spline.c.shape[1]):
        poly=PPoly.from_spline((spline.t,spline.c[:,axis],spline.k)).derivative(order)
        peak=0.
        for index,(left,right) in enumerate(zip(poly.x,poly.x[1:])):
            if right<=left:continue
            coeff=poly.c[:,index];derivative=np.polyder(coeff)
            roots=np.roots(np.trim_zeros(derivative,"f")) if np.any(derivative) else []
            times=[0.,right-left]+[float(z.real) for z in roots
                if abs(z.imag)<1e-8 and 0.<z.real<right-left]
            peak=max(peak,max(abs(float(np.polyval(coeff,t))) for t in times))
        result.append(peak)
    return np.asarray(result)


def fit_curve(points,velocity,acceleration,jerk):
    from .trajectory_artifact import validate_timed_points
    validate_timed_points(points)
    if len(points)<2:raise ValueError("Spline requires moving timed points")
    axes=len(points[0].q)+1
    limits=[np.asarray(v,dtype=float) for v in (velocity,acceleration,jerk)]
    if any(v.shape!=(axes,) or not np.all(np.isfinite(v)) or np.any(v<=0) for v in limits):
        raise ValueError("Explicit positive limits required for every arm and rail axis")
    x=np.asarray([p.time_s for p in points]);y=np.asarray([(*p.q,p.rail_m) for p in points])
    zeros=np.zeros(axes)
    spline=make_interp_spline(x,y,k=5,bc_type=([(1,zeros),(2,zeros)],[(1,zeros),(2,zeros)]))
    peaks=[extrema(spline,n) for n in (1,2,3)]
    scale=max(1.,*(float(np.max(peak/limit))**(1/order)
                   for order,(peak,limit) in enumerate(zip(peaks,limits),1)))
    scale*=1.+1e-9
    return dict(degree=5,knots=spline.t.tolist(),coefficients=spline.c.tolist(),
        source_duration_s=float(x[-1]),time_scale=scale,duration_s=float(x[-1])*scale,
        extrema_method="polynomial_roots_plus_interval_endpoints",
        maximum_velocity=(peaks[0]/scale).tolist(),
        maximum_acceleration=(peaks[1]/scale**2).tolist(),
        maximum_jerk=(peaks[2]/scale**3).tolist(),
        velocity_limits=limits[0].tolist(),acceleration_limits=limits[1].tolist(),
        jerk_limits=limits[2].tolist())



def fit_stopped_edges(points,velocity,acceleration,jerk):
    """C2 rest-to-rest quintic on each certified straight joint-space edge."""
    limits=[np.asarray(v,dtype=float) for v in (velocity,acceleration,jerk)]
    axes=len(points[0].q)+1
    if any(v.shape!=(axes,) or not np.isfinite(v).all() or np.any(v<=0) for v in limits):
        raise ValueError("Explicit positive axis limits required")
    times=[0.];coefficients=[]
    for left,right in zip(points,points[1:]):
        a=np.asarray([*left.q,left.rail_m]);b=np.asarray([*right.q,right.rail_m]);delta=np.abs(b-a)
        duration=max(1e-5,float(np.max(1.875*delta/limits[0])),
                     math.sqrt(float(np.max((10*math.sqrt(3)/3)*delta/limits[1]))),
                     float(np.max(60*delta/limits[2]))**(1/3))*(1+1e-9)
        times.append(times[-1]+duration)
        coefficients.extend([a,a,a,b,b,b])
    knots=np.repeat(times,6)
    spline=BSpline(knots,np.asarray(coefficients),5,extrapolate=False)
    peaks=[extrema(spline,n) for n in (1,2,3)]
    return dict(degree=5,knots=knots.tolist(),coefficients=np.asarray(coefficients).tolist(),
        source_duration_s=times[-1],time_scale=1.,duration_s=times[-1],
        extrema_method="polynomial_roots_plus_interval_endpoints",
        maximum_velocity=peaks[0].tolist(),maximum_acceleration=peaks[1].tolist(),maximum_jerk=peaks[2].tolist(),
        velocity_limits=limits[0].tolist(),acceleration_limits=limits[1].tolist(),jerk_limits=limits[2].tolist(),
        stop_times_s=times,intermediate_stops=max(0,len(points)-2))

def curve_spline(curve):
    return BSpline(np.asarray(curve["knots"]),np.asarray(curve["coefficients"]),curve["degree"],extrapolate=False)


def point_at(curve,local_time,global_time):
    scale=curve["time_scale"]
    t=min(max(local_time/scale,0.),curve["source_duration_s"])
    spline=curve_spline(curve)
    q=spline(t);v=spline(t,nu=1)/scale;a=spline(t,nu=2)/scale**2
    return PlanWaypoint(tuple(q[:-1]),float(q[-1]),global_time,"spline_candidate",
        velocities=tuple(v[:-1]),accelerations=tuple(a[:-1]),rail_velocity=float(v[-1]),
        rail_acceleration=float(a[-1]))


class CurvePoints:
    def __init__(self,points,segments):
        self.points=tuple(points);self.segments=segments
        self.starts=[s["start_time_s"] for s in segments]
    def __len__(self):return len(self.points)
    def __getitem__(self,index):return self.points[index]
    def sample(self,time_s):
        from .trajectory_artifact import sample
        if not math.isfinite(time_s) or time_s<0 or time_s>self.points[-1].time_s+1e-9:
            raise ValueError("Sample time outside artifact")
        segment=self.segments[max(0,bisect_right(self.starts,time_s)-1)]
        if time_s<=segment["end_time_s"]+1e-9:
            return point_at(segment["execution_curve"],time_s-segment["start_time_s"],time_s)
        held=sample(self.points,time_s)
        return replace(held,velocities=(0.,)*len(held.q),accelerations=(0.,)*len(held.q),
                       rail_velocity=0.,rail_acceleration=0.)


def build_smoothed_artifact(source,output,*,velocity,acceleration,jerk,knot_indices_by_segment=None,stop_at_selected_knots=False,curve_overrides=None):
    from .trajectory_artifact import read_artifact,_digest
    original,points=read_artifact(source)
    if original["schema"]!= "unilab.timed-motion/v1":raise ValueError("Smoothing requires the native source artifact")
    value=copy.deepcopy(original);value.pop("digest")
    value.update(schema=SCHEMA,interpolation=INTERPOLATION,
        source_native_digest=original["digest"],
        qualification={"state":"candidate_requires_revalidation","hardware":False,
            "reason":"Spline changes between-knot geometry; raw native collision verdict does not transfer"})
    if set(curve_overrides or {})-{s["id"] for s in original["segments"]}:
        raise ValueError("Curve override names an unknown segment")
    offset=0.
    for segment,raw in zip(value["segments"],original["segments"]):
        native=tuple(PlanWaypoint(**p) for p in raw["result"]["waypoints"])
        chosen=(knot_indices_by_segment or {}).get(segment["id"])
        fit_points=native
        if chosen is not None:
            if (len(chosen)<2 or chosen!=sorted(set(chosen)) or chosen[0]!=0
                    or chosen[-1]!=len(native)-1 or any(type(i) is not int for i in chosen)):
                raise ValueError("Shortcut knot indices must retain ordered native endpoints")
            selected=[native[i] for i in chosen]
            timed=[replace(selected[0],time_s=0.)]
            limits=np.asarray(velocity,dtype=float)
            for left,right in zip(selected,selected[1:]):
                distance=np.abs(np.asarray([*right.q,right.rail_m])-np.asarray([*left.q,left.rail_m]))
                duration=max(1e-5,float(np.max(distance/limits)))
                timed.append(replace(right,time_s=timed[-1].time_s+duration))
            fit_points=tuple(timed)
        override=(curve_overrides or {}).get(segment["id"])
        if override is not None:
            if chosen is not None:raise ValueError("Conflicting curve and shortcut selections")
            curve=copy.deepcopy(override)
            for t,reference in ((0.,native[0]),(curve["duration_s"],native[-1])):
                point=point_at(curve,t,t)
                if (max(abs(a-b) for a,b in zip(point.q,reference.q))>1e-8
                        or abs(point.rail_m-reference.rail_m)>1e-8):
                    raise ValueError("Derived curve endpoint differs from accepted native state")
            for order,label,required in ((1,"velocity",velocity),(2,"acceleration",acceleration),(3,"jerk",jerk)):
                if not np.allclose(curve[label+"_limits"],required,atol=0,rtol=0):
                    raise ValueError("Derived curve uses different axis limits")
                actual=extrema(curve_spline(curve),order)/curve["time_scale"]**order
                if not np.allclose(actual,curve["maximum_"+label],atol=1e-8,rtol=1e-8):
                    raise ValueError("Derived curve extrema differ")
                if np.any(actual>np.asarray(required)*(1+1e-8)):
                    raise ValueError("Derived curve exceeds axis limits")
        else:
            curve=(fit_stopped_edges(fit_points,velocity,acceleration,jerk)
                   if chosen is not None and stop_at_selected_knots
                   else fit_curve(fit_points,velocity,acceleration,jerk))
            curve["native_knot_indices"]=list(range(len(native))) if chosen is None else list(chosen)
            curve["knot_policy"]=("all_native_timed_points" if chosen is None else
                                  "collision_checked_subset_stopped_edges" if stop_at_selected_knots
                                  else "collision_checked_subset_retimed")
        segment["source_start_time_s"]=raw["start_time_s"]
        segment["source_end_time_s"]=raw["end_time_s"]
        segment["start_time_s"]=raw["start_time_s"]+offset
        segment["end_time_s"]=segment["start_time_s"]+curve["duration_s"]
        segment["execution_curve"]=curve
        offset+=curve["duration_s"]-native[-1].time_s
    def mapped(t):
        previous=None
        for segment in value["segments"]:
            if segment["source_start_time_s"]<=t<=segment["source_end_time_s"]+1e-9:
                fraction=(t-segment["source_start_time_s"])/(segment["source_end_time_s"]-segment["source_start_time_s"])
                return segment["start_time_s"]+fraction*(segment["end_time_s"]-segment["start_time_s"])
            if segment["source_end_time_s"]<t:previous=segment
        if previous:return previous["end_time_s"]+(t-previous["source_end_time_s"])
        return t
    for event in value["events"]:
        event["source_time_s"]=event["time_s"]
        event["time_s"]=mapped(event["time_s"])
        if "end_time_s" in event:event["end_time_s"]=mapped(event["end_time_s"])
    value["waypoints"]=[replace(p,time_s=mapped(p.time_s)).as_mapping() for p in points]
    curve_points=CurvePoints([PlanWaypoint(**p) for p in value["waypoints"]],value["segments"])
    value["waypoints"]=[curve_points.sample(p["time_s"]).as_mapping() for p in value["waypoints"]]
    value["digest"]=_digest(value)
    path=Path(output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False),encoding="utf8")
    return value


def position_ranges(curve):
    spline=curve_spline(curve);ranges=[]
    for axis in range(spline.c.shape[1]):
        poly=PPoly.from_spline((spline.t,spline.c[:,axis],spline.k))
        values=[]
        for index,(left,right) in enumerate(zip(poly.x,poly.x[1:])):
            if right<=left:continue
            coeff=poly.c[:,index];derivative=np.polyder(coeff)
            roots=np.roots(np.trim_zeros(derivative,"f")) if np.any(derivative) else []
            times=[0.,right-left]+[float(z.real) for z in roots if abs(z.imag)<1e-8 and 0.<z.real<right-left]
            values.extend(float(np.polyval(coeff,t)) for t in times)
        ranges.append((min(values),max(values)))
    return ranges


def validation_times(segment,*,joint_step_rad=.01,rail_step_m=.001,max_period_s=.1):
    """以解析速度界保证采样间轴位移上界；不是连续碰撞证明。"""
    curve=segment["execution_curve"];velocity=curve["maximum_velocity"]
    period=min(max_period_s,joint_step_rad/max(max(velocity[:-1]),1e-12),
               rail_step_m/max(velocity[-1],1e-12))
    count=max(1,math.ceil(curve["duration_s"]/period))
    times=set(np.linspace(0.,curve["duration_s"],count+1).tolist())
    times.update(min(curve["duration_s"],max(0.,x*curve["time_scale"])) for x in curve["knots"])
    return sorted(times)
