"""将完整状态快照装入公共规划合同，不包含设备名称、站点或抓取策略。"""
import copy
import numpy as np
from unilab_robot_contracts.point_set_validation import stable_digest
from unilab_robot_contracts.planning import PlanRequest,PlanConstraints
from .attachment_state import local_mounts

class SceneRequestBuilder:
    def __init__(self,cad,robot,rail_axis,link_pose_resolver,bindings):
        self.cad,self.robot,self.rail_axis=cad,robot,rail_axis
        self.link_pose_resolver=link_pose_resolver
        self.bindings=tuple(dict(b) for b in bindings)
        ids=[b["body"] for b in self.bindings]
        if len(set(ids))!=len(ids) or set(ids)-set(cad.bodies):
            raise ValueError("Unknown or duplicate fixed attachment binding")

    def build(self,state,goal,*,constraints=None,goal_rail_m=None,timeout_s=10.):
        if state.get("robot_model_digest")!=self.robot.digest:
            raise ValueError("State robot model digest differs")
        if not state.get("snapshot_id") or not state.get("source"):
            raise ValueError("Explicit current state identity/source required")
        joints=state["joint_positions"]
        expected=set(self.robot.joint_names)
        if self.rail_axis:expected.add(self.rail_axis.axis_id)
        if not expected<=set(joints):raise ValueError("State must cover all robot and rail joints")
        from .urdf_tree import complete_joint_positions
        passive={name:value for name,value in joints.items() if name not in expected}
        joints=complete_joint_positions(self.robot.urdf_xml,{name:joints[name] for name in expected},passive)
        q=self.robot.validate_joint_positions(tuple(joints[k] for k in self.robot.joint_names))
        rail=joints[self.rail_axis.axis_id] if self.rail_axis else None
        if self.rail_axis and not self.rail_axis.contains(rail):raise ValueError("Rail state outside limits")
        link_poses=self.link_pose_resolver(joints)
        body_poses=state["body_poses"]
        carried=tuple(state.get("carried_body_ids",()))
        held=state.get("held_material")
        if bool(carried)!=bool(held):raise ValueError("Carried bodies and held material must agree")
        if len(set(carried))!=len(carried):raise ValueError("Duplicate carried body")
        bindings=list(self.bindings)
        if set(carried)&{b["body"] for b in bindings}:raise ValueError("Fixture/tool cannot be carried payload")
        for body in carried:
            if self.cad.bodies.get(body,{}).get("role")!="material":raise ValueError("Unknown carried material")
            bindings.append(dict(body=body,link_name=self.robot.flange_link,touch_links=()))
        mounts=local_mounts(state,self.robot.urdf_xml,bindings,link_poses)
        attachments=[]
        for binding in bindings:
            body,link=binding["body"],binding["link_name"]
            if body not in body_poses:raise ValueError("Fresh poses must cover every moving CAD body exactly")
            if link not in link_poses:raise ValueError("Link pose resolver omitted attachment link")
            relative=(mounts[body] if mounts is not None else
                      np.linalg.inv(np.asarray(link_poses[link]))@np.asarray(body_poses[body]))
            if "expected_link_from_body" in binding and not np.allclose(
                    relative,binding["expected_link_from_body"],rtol=0,atol=1e-5):
                raise ValueError("Current attachment state disagrees with approved mount: "+body)
            attachments.append(dict(body=body,link_name=link,flange_from_body=relative.tolist(),
                touch_links=binding.get("touch_links",()),purpose=binding.get("purpose","tool_or_payload")))
        world,specs=self.cad.snapshot(body_poses,attachments,flange_link=self.robot.flange_link)
        return PlanRequest(robot=self.robot,start_q=q,goal=goal,rail_axis=self.rail_axis,
            start_rail_m=rail,goal_rail_m=goal_rail_m,collision_world=world,attachments=specs,
            constraints=constraints or PlanConstraints(),carried_body_ids=carried,held_material=held,
            timeout_s=timeout_s,options={"passive_joint_positions":{k:v for k,v in joints.items() if k not in expected},"scene_snapshot_id":state["snapshot_id"],"scene_state_source":state["source"],
                "scene_geometry_digest":self.cad.digest,
                "scene_state":dict(copy.deepcopy(state),state_digest=stable_digest(state))})
