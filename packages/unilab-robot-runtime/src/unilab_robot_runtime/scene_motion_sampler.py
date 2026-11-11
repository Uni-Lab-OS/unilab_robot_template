"""规划与预览共享的全场景采样器；保留原始时钟、载荷与夹指状态。"""
from bisect import bisect_right
import math
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .trajectory_artifact import read_artifact,sample


def interpolate_poses(left,right,fraction):
    if set(left)!=set(right):raise ValueError("Event body identities changed")
    result={}
    for name in left:
        a,b=np.asarray(left[name]),np.asarray(right[name])
        pose=np.eye(4)
        pose[:3,3]=(1-fraction)*a[:3,3]+fraction*b[:3,3]
        pose[:3,:3]=Slerp([0.,1.],Rotation.from_matrix([a[:3,:3],b[:3,:3]]))([fraction]).as_matrix()[0]
        result[name]=pose.tolist()
    return result


class SceneMotionSampler:
    def __init__(self,path,robot,rail_axis,link_poses):
        self.artifact,self.points=read_artifact(path)
        if self.artifact["robot_model_digest"]!=robot.digest:raise ValueError("Robot model identity differs")
        if self.artifact["joint_names"]!=list(robot.joint_names):raise ValueError("Joint order differs")
        self.robot,self.rail_axis,self.link_poses=robot,rail_axis,link_poses
        self.segments=self.artifact["segments"]
        self.starts=[s["start_time_s"] for s in self.segments]
        self.events=self.artifact["events"]
        previous_end=0.
        for event in self.events:
            start=event["time_s"];end=event.get("end_time_s",start)
            if (not all(math.isfinite(t) for t in (start,end))
                    or not previous_end<=start<=end<=self.points[-1].time_s):
                raise ValueError("Invalid or overlapping scene event interval")
            previous_end=end
            if end>start:
                if not event.get("before") or not event.get("after"):
                    raise ValueError("Timed scene event requires both endpoint states")
                samples=event.get("samples")
                if samples and (len(samples)<2 or abs(samples[0]["time_s"])>1e-9
                        or abs(samples[-1]["time_s"]-(end-start))>1e-8
                        or any(not math.isfinite(p["time_s"]) for p in samples)
                        or any(b["time_s"]<=a["time_s"] for a,b in zip(samples,samples[1:]))):
                    raise ValueError("Event samples must span its complete interval")
        for segment in self.segments:
            if not segment["scene_state"].get("body_poses"):
                raise ValueError("Artifact lacks full OS body-state provenance")

    def at(self,time_s):
        point=sample(self.points,time_s)
        positions=dict(zip(self.robot.joint_names,point.q))
        positions[self.rail_axis.axis_id]=point.rail_m
        links=self.link_poses(self.robot.urdf_xml,positions)
        segment=self.segments[max(0,bisect_right(self.starts,time_s)-1)]
        state=segment["scene_state"]
        bodies=dict(state["body_poses"])
        for attachment in segment["attachments"]:
            bodies[attachment["body_id"]]=(links[attachment["link_name"]]@
                np.asarray(attachment["transform"])).tolist()
        event=next((e for e in self.events if e.get("end_time_s",e["time_s"])>e["time_s"]
                    and e["time_s"]<=time_s<=e["end_time_s"]),None)
        label=segment["id"]
        if event:
            samples=event.get("samples") or [
                {"time_s":0.,"state":event["before"]},
                {"time_s":event["end_time_s"]-event["time_s"],"state":event["after"]}]
            local=time_s-event["time_s"]
            index=min(max(0,bisect_right([p["time_s"] for p in samples],local)-1),len(samples)-2)
            left,right=samples[index:index+2]
            fraction=(local-left["time_s"])/(right["time_s"]-left["time_s"])
            bodies=interpolate_poses(left["state"]["body_poses"],right["state"]["body_poses"],fraction)
            state=event["after"] if time_s>=event["end_time_s"] else event["before"]
            label=event["id"]
        result=dict(time_s=time_s,phase=label,joint_positions=positions,
                    robot_model_digest=self.robot.digest,
                    rail_pose_origin_m=state.get("rail_pose_origin_m",
                        state.get("joint_positions",{}).get(self.rail_axis.axis_id,
                            self.points[0].rail_m)),
                    body_poses=bodies,link_poses={k:v.tolist() for k,v in links.items()},
                    carried_body_ids=state.get("carried_body_ids",[]),
                    held_material=state.get("held_material"),
                    source_artifact_digest=self.artifact["digest"],hardware=False)
        # Interpolated finger events have changing local mounts; use legacy
        # reconstruction between samples, never attach an endpoint receipt to them.
        if not event and "attachment_state" in state:
            import copy
            result["attachment_state"]=copy.deepcopy(state["attachment_state"])
        return result

    def play(self, *, apply, observe, start_s=0., end_s=None, period_s=1/30):
        """独立场景运动学执行及逐采样读回；不声明物理或硬件资格。

        apply 消费与渲染相同的完整状态，observe 返回实际投影。
        每个原始路点和事件采样都保留；无起点补接、无隐式时间压缩。
        """
        end_s=self.points[-1].time_s if end_s is None else float(end_s)
        if (not all(math.isfinite(v) for v in (start_s,end_s,period_s))
                or not 0<=start_s<=end_s<=self.points[-1].time_s or period_s<=0):
            raise ValueError("Invalid playback interval or clock period")
        def verify(expected, actual):
            if actual.get("robot_model_digest") != self.robot.digest:
                raise ValueError("Execution robot model differs")
            a,b=expected["joint_positions"],actual["joint_positions"]
            if set(a)!=set(b) or any(not math.isfinite(b[n]) or abs(a[n]-b[n])>1e-6 for n in a):
                raise ValueError("Execution joint readback differs; no unplanned bridge")
            if (set(expected.get("carried_body_ids",()))!=set(actual.get("carried_body_ids",()))
                    or expected.get("held_material")!=actual.get("held_material")):
                raise ValueError("Execution custody readback differs")
            a,b=expected["body_poses"],actual["body_poses"]
            if set(a)!=set(b):
                raise ValueError("Execution body inventory differs")
            if any(not np.allclose(a[n],b[n],rtol=0,atol=1e-6) for n in a):
                raise ValueError("Execution body pose readback differs; no unplanned bridge")
        verify(self.at(start_s),observe())
        times={start_s,end_s}
        times.update(p.time_s for p in self.points if start_s<=p.time_s<=end_s)
        times.update(float(t) for t in np.arange(start_s,end_s,period_s))
        for event in self.events:
            times.update(t for t in (event["time_s"],event.get("end_time_s",event["time_s"]))
                         if start_s<=t<=end_s)
            times.update(event["time_s"]+p["time_s"] for p in event.get("samples",())
                         if start_s<=event["time_s"]+p["time_s"]<=end_s)
        completed=0
        for timestamp in sorted(times):
            expected=self.at(timestamp)
            apply(expected)
            verify(expected,observe())
            completed+=1
        return dict(status="passed",artifact_digest=self.artifact["digest"],
                    start_time_s=start_s,end_time_s=end_s,samples=completed,
                    execution="independent_kinematic_scene_readback",
                    physics=False,hardware=False)
