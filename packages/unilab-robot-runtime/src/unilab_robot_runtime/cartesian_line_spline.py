"""Retiming a constant-rail Cartesian line without inheriting native timing corners."""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from unilab_robot_contracts.planning import PlanWaypoint
from .urdf_kinematics import UrdfKinematics
from .trajectory_spline import fit_curve

def fit_cartesian_line(robot,rail_axis,native,velocity,acceleration,jerk,*,translation_limits,rotation_speed_limit,samples=61):
    if samples<9:raise ValueError("Insufficient Cartesian interpolation samples")
    rail=float(native[0].rail_m)
    if any(abs(p.rail_m-rail)>1e-8 for p in native):
        raise ValueError("Contact LIN retiming requires a locked rail")
    kin=UrdfKinematics(robot);external={rail_axis.axis_id:rail}
    first=kin.forward(native[0].q,external);last=kin.forward(native[-1].q,external)
    direction=last[:3,3]-first[:3,3];length=float(direction@direction)
    for point in native:
        xyz=kin.forward(point.q,external)[:3,3]
        fraction=np.clip((xyz-first[:3,3])@direction/length,0.,1.) if length>1e-20 else 0.
        if np.linalg.norm(xyz-(first[:3,3]+fraction*direction))>1e-4:
            raise ValueError("Source path was not a Cartesian line")
    distance=float(np.sqrt(length))
    limits=np.asarray(translation_limits,dtype=float)
    if limits.shape!=(3,) or not np.isfinite(limits).all() or np.any(limits<=0):
        raise ValueError("Explicit positive Cartesian velocity/acceleration/jerk limits required")
    if distance<1e-6:raise ValueError("Pure rotation needs a separate retiming policy")
    angle=float(Rotation.from_matrix(first[:3,:3].T@last[:3,:3]).magnitude())
    if not np.isfinite(rotation_speed_limit) or rotation_speed_limit<=0:
        raise ValueError("Explicit rotational speed limit required")
    speed=min(limits[0],distance*rotation_speed_limit/max(angle,1e-12))
    def ramp(v):
        return max(1.875*v/limits[1],np.sqrt((10*np.sqrt(3)/3)*v/limits[2]))
    if speed*ramp(speed)>distance:
        low,high=0.,speed
        for _ in range(64):
            mid=(low+high)/2
            if mid*ramp(mid)>distance:high=mid
            else:low=mid
        speed=low
    ramp_time=ramp(speed);cruise=max(0.,distance/speed-ramp_time)
    duration=2*ramp_time+cruise
    def integrated(u):return 2.5*u**4-3*u**5+u**6
    def travel(t):
        if t<ramp_time:return speed*ramp_time*integrated(t/ramp_time)
        if t<ramp_time+cruise:return speed*ramp_time/2+speed*(t-ramp_time)
        tau=t-ramp_time-cruise
        return speed*ramp_time/2+speed*cruise+speed*(tau-ramp_time*integrated(tau/ramp_time))
    rotation=Slerp([0,1],Rotation.from_matrix([first[:3,:3],last[:3,:3]]))
    points=[];previous=native[0].q
    times=np.unique([*np.linspace(0.,duration,samples),ramp_time,ramp_time+cruise])
    for index,t in enumerate(times):
        progress=float(np.clip(travel(t)/distance,0.,1.))
        target=np.eye(4);target[:3,3]=first[:3,3]+progress*direction
        target[:3,:3]=rotation([progress]).as_matrix()[0]
        q=(native[0].q if index==0 else native[-1].q if index==len(times)-1 else
           kin.inverse(target,previous,external,position_tolerance_m=1e-7,rotation_tolerance_rad=1e-7))
        if q is None:raise ValueError("Cartesian line branch IK failed")
        if max(abs(a-b) for a,b in zip(previous,q))>.25:
            raise ValueError("Cartesian IK branch is discontinuous")
        points.append(PlanWaypoint(tuple(q),rail,float(t),"cartesian_line_geometric_resampling"))
        previous=q
    curve=fit_curve(points,velocity,acceleration,jerk)
    from .trajectory_spline import curve_spline
    sampled_times=np.linspace(0.,curve["source_duration_s"],1201)
    values=curve_spline(curve)(sampled_times)
    xyz=np.array([kin.forward(row[:-1],{rail_axis.axis_id:float(row[-1])})[:3,3] for row in values])
    dt=sampled_times[1]-sampled_times[0]
    sampled_speed=float(np.linalg.norm(np.diff(xyz,axis=0)/dt,axis=1).max())/curve["time_scale"]
    sampled_acc=float(np.linalg.norm(np.diff(xyz,n=2,axis=0)/dt**2,axis=1).max())/curve["time_scale"]**2
    adjustment=max(1.,sampled_speed/limits[0],np.sqrt(sampled_acc/limits[1]))*(1+1e-3)
    curve["time_scale"]*=adjustment;curve["duration_s"]*=adjustment
    for order,label in ((1,"velocity"),(2,"acceleration"),(3,"jerk")):
        curve["maximum_"+label]=[v/adjustment**order for v in curve["maximum_"+label]]
    curve.update(knot_policy="Cartesian line with smooth velocity ramps, cruise and IK; requires native revalidation",
        cartesian_translation_limits=limits.tolist(),rotation_speed_limit=float(rotation_speed_limit),
        sampled_cartesian_speed=sampled_speed/adjustment,sampled_cartesian_acceleration=sampled_acc/adjustment**2,
        cartesian_derivative_qualification="1201 discrete FK samples with 0.1 percent time margin",
        cartesian_jerk_qualification="nominal scalar progress only; robot joint jerk bound is analytic",
        native_knot_indices=None,cartesian_samples=len(times),source_line_start=first.tolist(),
        source_line_end=last.tolist(),intermediate_stops=0)
    return curve
