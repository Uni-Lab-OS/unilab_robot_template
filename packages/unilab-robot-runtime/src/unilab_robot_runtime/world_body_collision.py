"""Exact world-body collision queries using temporary existing-shape probes.

MoveIt normally checks robot-vs-world, not world-vs-world. Each selected
world object is temporarily attached without touch exemptions, checked against
all remaining geometry and the parked robot, then restored. This is a query
only: it does not assign material custody or execute a mechanism.
The caller must serialize access to its planner instance.
"""
import copy


def check_world_bodies(planner, request, body_ids):
    ids=tuple(body_ids)
    known=set(request.collision_world.object_ids)
    attached={a.body_id for a in request.attachments}
    if not ids or len(set(ids))!=len(ids) or set(ids)-known or set(ids)&attached:
        raise ValueError("Probe requires unique, existing, unattached world objects")
    planner._load_scene(request)
    original=planner._read_scene()
    probe_link=_independent_probe_link(request,additional_occupied={
        a.link_name for a in original.robot_state.attached_collision_objects})
    original_world={o.id for o in original.world.collision_objects}
    original_attached={o.object.id for o in original.robot_state.attached_collision_objects}
    if set(ids)-original_world:raise ValueError("Probe object missing from native scene")
    original_key=planner._loaded_scene_key
    planner._loaded_scene_key=None
    m=planner._session.messages
    active=None
    def apply(attachment=None, matrix=None):
        message=m["ApplyPlanningScene"].Request()
        message.scene.is_diff=True;message.scene.robot_state.is_diff=True
        if attachment is not None:message.scene.robot_state.attached_collision_objects=[attachment]
        if matrix is not None:message.scene.allowed_collision_matrix=copy.deepcopy(matrix)
        if not getattr(planner._call("scene",message),"success",False):
            raise RuntimeError("Native collision probe mutation rejected")
    def detach(identity):
        item=m["AttachedCollisionObject"]()
        item.object.id=identity;item.object.operation=m["CollisionObject"].REMOVE
        apply(item)
        # Removing a body also removes its ACM entries.
        apply(matrix=original.allowed_collision_matrix)
    checks=[]
    try:
        for identity in ids:
            item=m["AttachedCollisionObject"]()
            item.link_name=probe_link
            item.object.id=identity;item.object.operation=m["CollisionObject"].ADD
            item.touch_links=[]
            active=identity
            apply(item)
            apply(matrix=original.allowed_collision_matrix)
            query=m["GetStateValidity"].Request()
            query.group_name=""
            query.robot_state=planner._state(request,request.start_q,request.start_rail_m)
            query.robot_state.is_diff=True
            response=planner._call("validity",query)
            row=dict(body_id=identity,valid=bool(response.valid),contacts=[
                dict(body_1=c.contact_body_1,body_2=c.contact_body_2,depth_m=float(c.depth))
                for c in response.contacts])
            checks.append(row)
            detach(identity);active=None
            if not row["valid"]:break
    finally:
        if active is not None:detach(active)
        restored=planner._read_scene()
        world={o.id for o in restored.world.collision_objects}
        attached_now={o.object.id for o in restored.robot_state.attached_collision_objects}
        if world!=original_world or attached_now!=original_attached:
            raise RuntimeError("Native collision probe custody restoration differs")
        if restored.allowed_collision_matrix!=original.allowed_collision_matrix:
            raise RuntimeError("Native collision probe ACM restoration differs")
        planner._loaded_scene_key=original_key
    return dict(valid=len(checks)==len(ids) and all(r["valid"] for r in checks),
        checks=checks,restored=True,geometry="existing exact native shapes",
        diagnostic_attachment_only=True,physical_contact_qualification="not_run")

def _independent_probe_link(request,*,additional_occupied=()):
    """A collision-free model link unused by existing attachments avoids FCL sibling exemptions."""
    import xml.etree.ElementTree as ET
    root=ET.fromstring(request.robot.urdf_xml)
    occupied={a.link_name for a in request.attachments}|set(additional_occupied)
    children={j.find("child").get("link") for j in root.findall("joint")}
    candidates=[link.get("name") for link in root.findall("link")
                if link.find("collision") is None and link.get("name") not in occupied]
    candidates.sort(key=lambda name:name in children)
    if not candidates:
        raise ValueError("Exact probe requires a collision-free URDF link unused by attachments")
    return candidates[0]


def check_attached_bodies(planner, request, body_ids=None):
    """Check attached assemblies, including siblings normally exempted by MoveIt FCL.

    Temporarily reparent one existing native shape to an independent collision-free
    link, preserving its world pose and declared touch links. Restore custody, local
    poses, shape counts, ACM and the native scene receipt before releasing the cache.
    Serial query only; no physical custody change. No successful-result cache.
    """
    specs={a.body_id:a for a in request.attachments}
    ids=tuple(specs if body_ids is None else body_ids)
    if not ids or len(set(ids))!=len(ids) or set(ids)-set(specs):
        raise ValueError("Probe requires unique existing attached objects")
    planner._load_scene(request)
    original=planner._read_scene()
    probe_link=_independent_probe_link(request,additional_occupied={
        a.link_name for a in original.robot_state.attached_collision_objects})
    native={a.object.id:a for a in original.robot_state.attached_collision_objects}
    if set(ids)-set(native):raise ValueError("Attached probe object missing in native scene")
    original_world={o.id for o in original.world.collision_objects}
    key=planner._loaded_scene_key
    planner._loaded_scene_key=None
    m=planner._session.messages
    active=None
    checks=[]
    def mutate(identity,operation,link="",touch=()):
        item=m["AttachedCollisionObject"]()
        item.object.id=identity;item.object.operation=operation
        item.link_name=link;item.touch_links=list(touch)
        query=m["ApplyPlanningScene"].Request()
        query.scene.is_diff=True;query.scene.robot_state.is_diff=True
        query.scene.robot_state.attached_collision_objects=[item]
        if not planner._call("scene",query).success:raise RuntimeError("Attached probe mutation rejected")
        # Detachment removes ACM entries, so restore the complete original matrix after each mutation.
        acm=m["ApplyPlanningScene"].Request()
        acm.scene.is_diff=True;acm.scene.robot_state.is_diff=True
        acm.scene.allowed_collision_matrix=copy.deepcopy(original.allowed_collision_matrix)
        if not planner._call("scene",acm).success:raise RuntimeError("Attached probe ACM restoration rejected")
    def restore_active(*,known_attached=False):
        nonlocal active
        if active is None:return
        if known_attached:
            mutate(active,m["CollisionObject"].REMOVE)
        else:
            # Only an interrupted mutation needs an extra native custody read.
            current=planner._read_scene()
            attached={a.object.id:a for a in current.robot_state.attached_collision_objects}
            if active in attached:mutate(active,m["CollisionObject"].REMOVE)
        old=native[active]
        mutate(active,m["CollisionObject"].ADD,old.link_name,old.touch_links)
        active=None
    def pose_values(pose):
        return [pose.position.x,pose.position.y,pose.position.z,
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    try:
        for identity in ids:
            active=identity
            mutate(identity,m["CollisionObject"].REMOVE)
            mutate(identity,m["CollisionObject"].ADD,probe_link,native[identity].touch_links)
            query=m["GetStateValidity"].Request()
            query.group_name=""
            query.robot_state=planner._state(request,request.start_q,request.start_rail_m)
            query.robot_state.is_diff=True
            response=planner._call("validity",query)
            row=dict(body_id=identity,valid=bool(response.valid),contacts=[
                dict(body_1=c.contact_body_1,body_2=c.contact_body_2,depth_m=float(c.depth))
                for c in response.contacts])
            checks.append(row)
            restore_active(known_attached=True)
            if not row["valid"]:break
    finally:
        restore_active()
        restored=planner._read_scene()
        current={a.object.id:a for a in restored.robot_state.attached_collision_objects}
        if {o.id for o in restored.world.collision_objects}!=original_world or set(current)!=set(native):
            raise RuntimeError("Attached probe custody restoration differs")
        if restored.allowed_collision_matrix!=original.allowed_collision_matrix or restored.name!=original.name:
            raise RuntimeError("Attached probe ACM/receipt restoration differs")
        for identity,old in native.items():
            new=current[identity]
            if old.link_name!=new.link_name or set(old.touch_links)!=set(new.touch_links):
                raise RuntimeError("Attached probe binding restoration differs")
            a,b=old.object,new.object
            if any(len(getattr(a,k))!=len(getattr(b,k)) for k in ("meshes","primitives","planes")):
                raise RuntimeError("Attached probe shape inventory differs")
            old_poses=[a.pose,*a.mesh_poses,*a.primitive_poses,*a.plane_poses]
            new_poses=[b.pose,*b.mesh_poses,*b.primitive_poses,*b.plane_poses]
            if len(old_poses)!=len(new_poses) or any(
                abs(x-y)>1e-9 for pa,pb in zip(old_poses,new_poses)
                for x,y in zip(pose_values(pa),pose_values(pb))):
                raise RuntimeError("Attached probe local pose restoration differs")
        planner._loaded_scene_key=key
    return dict(valid=len(checks)==len(ids) and all(c["valid"] for c in checks),
                checks=checks,restored=True,probe_link=probe_link,
                geometry="existing exact native shapes",same_link_pairs_checked=True,
                physical_contact_qualification="not_run")

def check_scene_state(planner, request, *, q=None, rail_m=None, world_body_ids=()):
    """同步并校验一个状态的机器人、世界运动体与所有附件。

    world_body_ids 由调用者明确声明，不能据“机器人无碰撞”推断世界体有效。
    查询端口只限同一独占 PlanningScene；失败和探针恢复异常直接传播。
    """
    from dataclasses import replace
    sampled = replace(request,
        start_q=tuple(request.start_q if q is None else q),
        start_rail_m=request.start_rail_m if rail_m is None else float(rail_m))
    ids = tuple(world_body_ids)
    known = set(sampled.collision_world.object_ids)
    attached_ids = {a.body_id for a in sampled.attachments}
    if len(ids) != len(set(ids)) or set(ids)-known or set(ids)&attached_ids:
        raise ValueError("Expected unique unattached world-body identities")
    planner._load_scene(sampled)
    robot = planner._state_validity(sampled, sampled.start_q,
                                    sampled.start_rail_m, whole_robot=True)
    report = dict(valid=False, robot=robot, world=None, attachments=None,
                  coverage=["robot_world"])
    if not robot["valid"]:
        return report
    if ids:
        report["world"] = check_world_bodies(planner, sampled, ids)
        report["coverage"].extend(["moving_world", "moving_moving"])
        if not report["world"]["valid"]:
            return report
    else:
        # 调用者明确没有世界运动体；不隐藏缺省选择。
        report["coverage"].extend(["moving_world", "moving_moving"])
    if sampled.attachments:
        report["attachments"] = check_attached_bodies(planner, sampled)
        report["coverage"].append("attachments")
        if not report["attachments"]["valid"]:
            return report
    else:
        report["coverage"].append("attachments")
    report["valid"] = True
    return report
