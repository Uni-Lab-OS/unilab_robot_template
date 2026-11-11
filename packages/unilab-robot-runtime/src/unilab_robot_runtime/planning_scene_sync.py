"""Transactional, package-neutral MoveIt scene synchronization.

Preserve SRDF/external ACM entries, scope request exemptions to this request,
and fail closed if ApplyPlanningScene or the ownership readback disagrees.
A planner instance requires exclusive ownership of its object ids.
"""
from __future__ import annotations
import copy
import uuid

def _value(matrix, a, b):
    names = list(matrix.entry_names)
    if a in names and b in names:
        return bool(matrix.entry_values[names.index(a)].enabled[names.index(b)])
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    if a in defaults and b in defaults:
        return bool(defaults[a] and defaults[b])
    return bool(defaults.get(a, defaults.get(b, False)))

def _set(matrix, a, b, value, entry_type):
    for name in (a, b):
        if name not in matrix.entry_names:
            old = list(matrix.entry_names)
            values = [_value(matrix, name, other) for other in old]
            for row, enabled in zip(matrix.entry_values, values):
                row.enabled.append(enabled)
            diagonal = _value(matrix, name, name)
            matrix.entry_names.append(name)
            entry = entry_type()
            entry.enabled = values + [diagonal]
            matrix.entry_values.append(entry)
    i, j = matrix.entry_names.index(a), matrix.entry_names.index(b)
    matrix.entry_values[i].enabled[j] = bool(value)
    matrix.entry_values[j].enabled[i] = bool(value)

def sync_robot_state(planner, request, *, q=None, rail=None):
    """同步完整机器人状态；Pilz 的组外轴依赖 PlanningScene 当前值。"""
    m=planner._session.messages
    state=planner._state(request,request.start_q if q is None else q,
                         request.start_rail_m if rail is None else rail)
    state.is_diff=True
    message=m["ApplyPlanningScene"].Request()
    message.scene.is_diff=True
    message.scene.robot_state=state
    if not getattr(planner._call("scene",message),"success",False):
        raise RuntimeError("PlanningScene rejected current robot state")
    observed_scene=planner._read_scene(state_only=True)
    receipt=getattr(planner,"_scene_receipt",None)
    if receipt is not None and observed_scene.name != receipt:
        # A restarted/replaced scene cannot inherit the client's previous acknowledgment.
        planner._loaded_scene_key=None
        planner._scene_receipt=None
    actual=observed_scene.robot_state.joint_state
    observed=dict(zip(actual.name,actual.position))
    expected=dict(zip(state.joint_state.name,state.joint_state.position))
    if any(name not in observed or abs(observed[name]-value)>1e-8 for name,value in expected.items()):
        raise RuntimeError("PlanningScene current robot state readback mismatch")
    return {"verified":True,"joint_positions":expected}


def _same_geometry(previous, current):
    return (previous is not None
            and len(previous.geometry) == len(current.geometry)
            and all(a is b for a, b in zip(previous.geometry, current.geometry))
            and previous.geometry_poses == current.geometry_poses)


def apply_scene(planner, request):
    from .moveit_move_group import _object_to_native, _matrix_to_native
    robot_state = sync_robot_state(planner, request)
    key = planner._scene_key(request)
    if planner._loaded_scene_key == key:
        return {"reloaded": False, "verified": True, "robot_state": robot_state}
    if request.collision_world.frame_id != "world":
        raise ValueError("Collision world must use the robot physical world frame")
    m = planner._session.messages
    by_id = {obj.object_id: obj for obj in request.collision_world.objects}
    attachments = {a.body_id: a for a in request.attachments}
    wanted_attached = set(attachments)
    wanted_world = set(by_id) - wanted_attached
    current_pairs = set(request.constraints.allowed_collision_pairs)
    # 附着体的世界位姿随关节变化；原生场景用 link + local pose 表达它。
    # 只变世界投影不代表几何/安装关系变化。失败恢复期间不得走此缓存。
    equivalent = (planner._loaded_scene_key is not None
                  and wanted_world == planner._loaded_ids
                  and wanted_attached == planner._attached_ids
                  and attachments == getattr(planner, "_loaded_attachment_specs", {})
                  and current_pairs == planner._allowed_pairs
                  and all(_same_geometry(planner._loaded_objects.get(identity), obj)
                          and (identity in attachments
                               or planner._loaded_objects[identity].world_pose == obj.world_pose)
                          for identity, obj in by_id.items()))
    if equivalent:
        planner._loaded_scene_key = key
        planner._loaded_objects = by_id
        return {"reloaded": False, "verified": True, "robot_state": robot_state,
                "equivalent_native_scene": True, "scene_digest": key[0]}
    original = planner._read_scene()
    matrix = copy.deepcopy(original.allowed_collision_matrix)
    restored = dict(planner._acm_original)
    for pair, value in restored.items():
        _set(matrix, *pair, value, m["AllowedCollisionEntry"])
    saved = {pair: _value(matrix, *pair) for pair in current_pairs}
    for pair in current_pairs:
        _set(matrix, *pair, True, m["AllowedCollisionEntry"])

    def scene():
        value = m["PlanningScene"]()
        value.is_diff = True
        value.robot_state.is_diff = True
        return value

    def send(value):
        req = m["ApplyPlanningScene"].Request()
        req.scene = value
        response = planner._call("scene", req)
        if not getattr(response, "success", False):
            raise RuntimeError("ApplyPlanningScene rejected scene; planning prohibited")

    # Invalidate before any mutation; retries reassert all ids after a partial failure.
    planner._loaded_scene_key = None
    planner._scene_receipt = None
    pending = scene()
    native_world={obj.id for obj in original.world.collision_objects}
    native_attached={obj.object.id for obj in original.robot_state.attached_collision_objects}
    previous_attachments = getattr(planner, "_loaded_attachment_specs", {})
    retained = {identity for identity, spec in attachments.items()
                if identity in native_attached
                and previous_attachments.get(identity) == spec
                and _same_geometry(planner._loaded_objects.get(identity), by_id[identity])}
    detach=(planner._attached_ids & native_attached) - retained
    reusable = {identity for identity in wanted_attached - retained
                if identity in native_world | detach
                and _same_geometry(planner._loaded_objects.get(identity), by_id[identity])}
    for identity in sorted(detach):
        old = m["AttachedCollisionObject"]()
        old.object.id = identity
        old.object.operation = m["CollisionObject"].REMOVE
        pending.robot_state.attached_collision_objects.append(old)
    for identity in sorted((((planner._loaded_ids - wanted_world) | wanted_attached) & (native_world | detach)) - reusable):
        old = m["CollisionObject"]()
        old.id = identity
        old.operation = m["CollisionObject"].REMOVE
        pending.world.collision_objects.append(old)
    # Record union for recovery even if a transport times out after server mutation.
    planner._loaded_ids |= wanted_world
    planner._attached_ids |= wanted_attached
    planner._acm_original = {**saved, **restored}
    try:
        send(pending)
        pending = scene()
        for identity in sorted(wanted_world):
            obj = by_id[identity]
            previous = planner._loaded_objects.get(identity)
            unchanged_geometry = _same_geometry(previous, obj)
            if unchanged_geometry and identity in (native_world | detach):
                if identity not in detach and previous.world_pose == obj.world_pose:
                    continue
                native = m["CollisionObject"]()
                native.id = identity
                native.header.frame_id = "world"
                native.pose = _matrix_to_native(obj.world_pose)
                native.operation = m["CollisionObject"].MOVE
            else:
                native = _object_to_native(obj)
            pending.world.collision_objects.append(native)
            if len(pending.world.collision_objects) >= 128:
                send(pending)
                pending = scene()
        # MOVE 必须先独立提交：PlanningScene 先处理 robot_state，再处理 world。
        # 分离后的世界对象仍持有同一 shapes；重新附着时让原生端转移它，
        # 避免序列化网格和每个执行线程重新建立 FCL BVH。
        for identity in sorted(reusable):
            native = m["CollisionObject"]()
            native.id = identity
            native.operation = m["CollisionObject"].MOVE
            native.header.frame_id = attachments[identity].link_name
            native.pose = _matrix_to_native(attachments[identity].flange_from_body)
            pending.world.collision_objects.append(native)
        send(pending)
        pending = scene()
        for identity, attachment in attachments.items():
            if identity in retained:
                continue
            if identity in reusable:
                native = m["CollisionObject"]()
                native.id = identity
                native.operation = m["CollisionObject"].ADD
                # 无几何 ADD 从世界转移已有对象；位姿已在上一事务中更新。
            else:
                native = _object_to_native(by_id[identity])
                native.header.frame_id = attachment.link_name
                native.pose = _matrix_to_native(attachment.flange_from_body)
            attached = m["AttachedCollisionObject"]()
            attached.link_name = attachment.link_name
            attached.object = native
            attached.touch_links = list(attachment.touch_links)
            pending.robot_state.attached_collision_objects.append(attached)
        send(pending)
        # MoveIt 会在分离附着体时修改 ACM；所有对象操作完成后再提交最终矩阵。
        final_matrix=scene()
        final_matrix.allowed_collision_matrix=matrix
        receipt="unilab-scene:"+uuid.uuid4().hex
        final_matrix.name=receipt
        send(final_matrix)
        actual = planner._read_scene()
        if actual.name != receipt:
            raise RuntimeError("PlanningScene acknowledgment readback mismatch")
        actual_world = {obj.id for obj in actual.world.collision_objects}
        actual_attached = {obj.object.id: obj for obj in actual.robot_state.attached_collision_objects}
        owned = planner._loaded_ids | planner._attached_ids | set(planner._loaded_objects)
        if actual_world & owned != wanted_world or set(actual_attached) & owned != wanted_attached:
            raise RuntimeError("PlanningScene world/attachment ownership readback mismatch")
        for identity, attachment in attachments.items():
            observed = actual_attached[identity]
            if observed.link_name != attachment.link_name or set(observed.touch_links) != set(attachment.touch_links):
                raise RuntimeError("PlanningScene attachment link readback mismatch")
            expected_pose = _matrix_to_native(attachment.flange_from_body)
            observed_pose = observed.object.pose
            if any(abs(getattr(observed_pose.position, axis) - getattr(expected_pose.position, axis)) > 1e-9
                   for axis in ("x", "y", "z")):
                raise RuntimeError("PlanningScene attachment position readback mismatch")
            actual_q = [getattr(observed_pose.orientation, axis) for axis in ("x", "y", "z", "w")]
            expected_q = [getattr(expected_pose.orientation, axis) for axis in ("x", "y", "z", "w")]
            if min(max(abs(a-b) for a,b in zip(actual_q,expected_q)),
                   max(abs(a+b) for a,b in zip(actual_q,expected_q))) > 1e-9:
                raise RuntimeError("PlanningScene attachment rotation readback mismatch")
            shape_count = len(observed.object.meshes) + len(observed.object.primitives)
            if shape_count != len(by_id[identity].geometry):
                raise RuntimeError("PlanningScene attachment shape count readback mismatch")
        expected_values = {**restored, **{pair: True for pair in current_pairs}}
        if any(_value(actual.allowed_collision_matrix, *pair) != value
               for pair, value in expected_values.items()):
            differences=[(pair,value,_value(actual.allowed_collision_matrix,*pair))
                         for pair,value in expected_values.items()
                         if _value(actual.allowed_collision_matrix,*pair)!=value]
            raise RuntimeError("PlanningScene ACM readback mismatch: "+repr(differences))
    except Exception:
        planner._loaded_objects = {}
        raise
    planner._loaded_ids = wanted_world
    planner._attached_ids = wanted_attached
    planner._loaded_objects = by_id
    planner._loaded_attachment_specs = attachments
    planner._allowed_pairs = current_pairs
    planner._acm_original = saved
    planner._loaded_scene_key = key
    planner._scene_receipt = receipt
    return {"reloaded": True, "applied": True, "verified": True,
            "world_objects": len(wanted_world), "attached": len(wanted_attached),
            "native_scene_receipt": receipt,
            "retained_attachments": len(retained),
            "reused_attachment_geometry": len(reusable),
            "allowed_collision_pairs": sorted(current_pairs),
            "scene_digest": key[0], "robot_state": robot_state}
