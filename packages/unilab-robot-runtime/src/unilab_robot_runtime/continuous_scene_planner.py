"""Stateful planning session: fresh scene per segment, lossless output, no hidden reset."""
from dataclasses import replace
from pathlib import Path
import json
import logging
from .trajectory_artifact import TimedMotionArtifact
from .urdf_kinematics import UrdfKinematics
from unilab_robot_contracts.planning import PlanGoal

class MotionRejected(RuntimeError):
    def __init__(self,phase,result):
        self.phase,self.result=phase,result
        super().__init__(phase+": "+str(result.diagnostics.get("failure_category",result.status)))

class ContinuousScenePlanner:
    def __init__(self,provider,candidate,planner,output):
        self.provider,self.candidate,self.planner=provider,candidate,planner
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.artifact=TimedMotionArtifact(robot_model_digest=provider.robot.digest,
            scene_geometry_digest=provider.cad.digest,joint_names=provider.robot.joint_names)
        self.attempts=[]

    def motion(self,phase,goal,*,rail_m=None,contacts=(),orientation=(),
               primitive="joint_ptp",velocity_scale=.2,acceleration_scale=.2,
               pipelines=None):
        state=self.candidate.capture()
        actual_rail=state["joint_positions"][self.provider.rail_axis.axis_id]
        if primitive!="rail":
            if rail_m is not None and abs(float(rail_m)-actual_rail)>1e-4:
                raise ValueError("Arm phase requires an explicitly planned rail transfer first")
            # Freeze the observed rail endpoint, never the nominal commanded value.
            rail_m=actual_rail
        request=self.provider.request(state,goal,phase_contacts=contacts,orientation=orientation,
            goal_rail_m=rail_m,timeout_s=15.)
        if pipelines is None:
            pipelines=(("pilz","LIN"),) if primitive=="cartesian_linear" else (
                (("pilz","PTP"),("ompl","RRTConnectkConfigDefault")) if primitive=="rail" else
                (("pilz","PTP"),("ompl","RRTConnectkConfigDefault")))
        for index,(pipeline,planner_id) in enumerate(pipelines):
            configured=replace(request,options=dict(request.options,primitive=primitive,
                pipeline=pipeline,planner_id=planner_id,velocity_scale=velocity_scale,
                acceleration_scale=acceleration_scale))
            if pipeline=="ompl" and goal.kind=="pose":
                # Enumerate native IK branches in the already verified current scene.
                self.planner._load_scene(configured)
                probe=self.planner._compute_ik(configured)
                import math
                candidates=[]
                for sample in probe["samples"]:
                    if not sample.get("solution_validity",{}).get("valid"):continue
                    row=[]
                    for value,start,bounds in zip(sample["solution_q"],configured.start_q,
                                                   self.provider.robot.joint_limits):
                        legal=[value+2*math.pi*k for k in range(-2,3)
                               if bounds[0]<=value+2*math.pi*k<=bounds[1]]
                        if not legal:break
                        row.append(min(legal,key=lambda v:abs(v-start)))
                    if len(row)!=len(configured.start_q):continue
                    validity=self.planner._state_validity(configured,row,configured.goal_rail,whole_robot=True)
                    if validity["valid"]:candidates.append(tuple(row))
                if not candidates:raise ValueError("No collision-valid IK branch for Cartesian transit")
                q=min(candidates,key=lambda row:sum(abs(a-b) for a,b in zip(row,configured.start_q)))
                configured=replace(configured,goal=PlanGoal.from_joint(q),
                    options=dict(configured.options,cartesian_intent=goal.as_mapping(),
                                 goal_branch_source="native_collision_valid_nearest_winding",
                                 ik_branch_evidence=probe))
            result=self.planner.plan(configured)
            record=dict(phase=phase,attempt=index,pipeline=pipeline,planner_id=planner_id,
                        result=result.as_mapping(),request_digest=configured.digest)
            self.attempts.append(record)
            (self.output/"attempts.json").write_text(json.dumps(self.attempts,indent=2),encoding="utf8")
            logging.getLogger(__name__).info("motion %s %s %s %d",phase,pipeline,result.status,len(result.waypoints))
            if result.status=="success":
                self.artifact.append(phase,configured,result)
                self.candidate.accept_endpoint(result.waypoints[-1])
                self.artifact.save(self.output/"motion.json")
                return result
        raise MotionRejected(phase,result)

    def event(self,event_id,kind,before,after,*,duration_s,samples=()):
        if duration_s<=0:raise ValueError("State-changing events need explicit positive duration")
        if not self.artifact.points:raise ValueError("Event requires initial state")
        start=self.artifact.points[-1].time_s
        end=replace(self.artifact.points[-1],time_s=start+duration_s,source="event_hold")
        self.artifact.points.append(end)
        self.artifact.events.append(dict(id=event_id,kind=kind,time_s=start,end_time_s=end.time_s,
            before=before,after=after,interpolation="explicit_OS_body_state_transition",
            samples=list(samples),physical_contact_qualified=False))
        self.artifact.save(self.output/"motion.json")
