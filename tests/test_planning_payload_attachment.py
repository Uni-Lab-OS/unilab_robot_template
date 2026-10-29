"""P18：公共 planning 合同携带夹持载荷（``PayloadCollisionProfile``）的红测试。

本文件锁定 P18 的第一组不变量（先红后绿）：

* ``PlanRequest`` 可以携带一个已附着到法兰的载荷；没有载荷时请求的
  规范映射与摘要**逐位不变**（``payload=None`` 与省略该字段等价）；
* 携带载荷时，碰撞世界必须真的包含该载荷对象、且对象几何必须与
  ``PayloadCollisionProfile`` 一致；无法用 ``CollisionGeometry`` 表达的
  Profile（球/锥、带局部位姿的基元、只有 ``collision_asset_ref``）**显式
  拒绝**，不做静默降级；
* ``PlanResult`` 必须回显 payload identity / profile digest / attachment
  digest；缺失、部分、或与请求不一致都拒绝；
* 相邻段端点关节连续（``max |Δq|`` 上限 + 无 2π 整圈重标定），不连续
  不得被当成可执行轨迹。
"""

from __future__ import annotations

import math

import pytest

from unilab_robot_contracts import (
    AttachmentSpec,
    CollisionGeometry,
    CollisionObject,
    CollisionWorld,
    CollisionPrimitive,
    JointContinuityLimits,
    PayloadAttachment,
    PayloadCollisionProfile,
    PlanGoal,
    PlanRequest,
    PlanResult,
    PlanWaypoint,
    PlannerIdentity,
    RigidTransform,
    RobotModel,
    check_joint_continuity,
)

JOINTS = tuple(f"robot_cr5_joint_{index}" for index in range(1, 7))
FLANGE = "link6"
PAYLOAD_BODY = "payload/beaker-500ml"
PROFILE_REF = "szlab_beaker_500ml@v1"
PROFILE_DIGEST = "c" * 64
IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)
#: TCP 到载荷根（载荷体坐标系）的实测偏移（P13 ``tcp - beaker = +0.090944 m``）。
FLANGE_FROM_BODY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, -0.031944),
    (0.0, 0.0, 0.0, 1.0),
)
CYLINDER = CollisionPrimitive(
    "cylinder",
    (0.118, 0.0483),
    RigidTransform.identity(),
)


def _urdf() -> str:
    joints = "".join(
        f"""
  <joint name="{name}" type="revolute">
    <parent link="{'base' if index == 1 else f'link{index - 1}'}"/>
    <child link="link{index}"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>"""
        for index, name in enumerate(JOINTS, start=1)
    )
    links = "".join(
        f'<link name="{f"base" if index == 0 else f"link{index}"}"/>'
        for index in range(7)
    )
    return f'<robot name="test_arm">{links}{joints}</robot>'


def _srdf() -> str:
    joints = "".join(f'<joint name="{name}"/>' for name in JOINTS)
    return f'<robot name="test_arm"><group name="robot_cr5_arm">{joints}</group></robot>'


def _model() -> RobotModel:
    return RobotModel(
        model_id="test-cr5",
        urdf_xml=_urdf(),
        srdf_xml=_srdf(),
        base_frame="base",
        flange_link=FLANGE,
        planning_group="robot_cr5_arm",
        joint_names=JOINTS,
        joint_limits=((-3.14, 3.14),) * 6,
        kinematics={"robot_cr5_arm": {"kinematics_solver": "test/KDL"}},
    )


def _profile(**overrides: object) -> PayloadCollisionProfile:
    """返回一个可进入规划碰撞世界的保守圆柱载荷。"""

    values: dict[str, object] = {
        "profile_ref": PROFILE_REF,
        "digest": PROFILE_DIGEST,
        "mass_kg": 0.2,
        "center_of_mass_m": (0.0, 0.0, 0.0),
        "root_to_grasp": RigidTransform((0.0, 0.0, 0.031944), (0.0, 0.0, 0.0, 1.0)),
        "collision_primitives": (CYLINDER,),
    }
    values.update(overrides)
    return PayloadCollisionProfile(**values)  # type: ignore[arg-type]


def _attachment(**overrides: object) -> AttachmentSpec:
    values: dict[str, object] = {
        "body_id": PAYLOAD_BODY,
        "link_name": FLANGE,
        "flange_from_body": FLANGE_FROM_BODY,
        "touch_links": ("link5",),
    }
    values.update(overrides)
    return AttachmentSpec(**values)  # type: ignore[arg-type]


def _payload(**overrides: object) -> PayloadAttachment:
    values: dict[str, object] = {"attachment": _attachment(), "profile": _profile()}
    values.update(overrides)
    return PayloadAttachment(**values)  # type: ignore[arg-type]


def _payload_object(**overrides: object) -> CollisionObject:
    values: dict[str, object] = {
        "object_id": PAYLOAD_BODY,
        "world_pose": FLANGE_FROM_BODY,
        "geometry": (
            CollisionGeometry("cylinder", radius_m=0.0483, length_m=0.118),
        ),
        "role": "payload",
        "attached_to_link": FLANGE,
        "touch_links": ("link5",),
    }
    values.update(overrides)
    return CollisionObject(**values)  # type: ignore[arg-type]


def _world(*extra: CollisionObject) -> CollisionWorld:
    return CollisionWorld(
        objects=(
            CollisionObject(
                "table",
                IDENTITY,
                (CollisionGeometry("box", size_m=(1.0, 1.0, 0.1)),),
                role="static_world",
            ),
            *extra,
        )
    )


def _request(**overrides: object) -> PlanRequest:
    values: dict[str, object] = {
        "robot": _model(),
        "start_q": (0.0,) * 6,
        "goal": PlanGoal.from_joint((0.1,) * 6),
        "collision_world": _world(),
    }
    values.update(overrides)
    return PlanRequest(**values)  # type: ignore[arg-type]


def _payload_request(**overrides: object) -> PlanRequest:
    values: dict[str, object] = {
        "collision_world": _world(_payload_object()),
        "attachments": (_attachment(),),
        "carried_body_ids": (PAYLOAD_BODY,),
        "held_material": PROFILE_REF,
        "payload": _payload(),
    }
    values.update(overrides)
    return _request(**values)


def _identity() -> PlannerIdentity:
    return PlannerIdentity("fake.ompl", "recording_fake", "v1.0.0")


def _result(request: PlanRequest, **overrides: object) -> PlanResult:
    values: dict[str, object] = {
        "status": "success",
        "waypoints": (
            PlanWaypoint(request.start_q, 0.0, 0.0),
            PlanWaypoint((0.1,) * 6, 0.0, 1.0),
        ),
        "planner": _identity(),
        "collision_checks": 1,
    }
    values.update(overrides)
    return PlanResult(**values)  # type: ignore[arg-type]


# --------------------------------------------------------------- 无载荷行为不变


def test_request_without_payload_is_unchanged() -> None:
    """没有载荷时规范映射不出现 payload 键，摘要与对照请求完全一致。"""

    request = _request()
    mapping = request.as_mapping()

    assert request.payload is None
    assert "payload" not in mapping
    assert request.payload_fields == {}
    assert request.digest == _request().digest
    # 省略与显式 None 必须等价（dataclass 默认值语义）。
    assert _request(payload=None).digest == request.digest


def test_result_without_payload_is_unchanged() -> None:
    """没有 payload 回显时结果映射不出现 payload 键，摘要保持原口径。"""

    request = _request()
    result = _result(request, payload_ref=None, payload_profile_digest=None,
                     attachment_digest=None)

    assert "payload" not in result.as_mapping()
    assert result.digest == _result(request).digest
    result.validate_against(request)


# ------------------------------------------------------------------ 携带载荷


def test_request_carries_payload_identity_profile_and_digest() -> None:
    """请求把载荷身份、profile digest 和 attachment digest 全部写进规范映射。"""

    payload = _payload()
    request = _payload_request(payload=payload)
    mapping = request.as_mapping()

    assert request.payload is payload
    assert payload.body_id == PAYLOAD_BODY
    assert payload.payload_ref == PROFILE_REF
    assert payload.profile_digest == PROFILE_DIGEST
    assert len(payload.digest) == 64
    assert mapping["payload"] == {
        "payload_ref": PROFILE_REF,
        "profile_digest": PROFILE_DIGEST,
        "attachment_digest": payload.digest,
        "body_id": PAYLOAD_BODY,
        "link_name": FLANGE,
    }
    assert request.payload_fields == {
        "payload_ref": PROFILE_REF,
        "payload_profile_digest": PROFILE_DIGEST,
        "attachment_digest": payload.digest,
    }
    # 载荷进入请求身份：同一世界/目标下，带载荷与不带载荷不是同一个请求。
    assert request.digest != _request().digest
    # 载荷几何是 Profile 的直接翻译（圆柱 length/radius 与 MoveIt 顺序一致）。
    assert payload.profile_geometries == (
        CollisionGeometry("cylinder", radius_m=0.0483, length_m=0.118),
    )


def test_payload_geometry_must_be_present_in_the_collision_world() -> None:
    """载荷对象必须真在碰撞世界里、标记为附着到法兰、且几何与 Profile 一致。"""

    with pytest.raises(ValueError, match="碰撞世界"):
        _payload_request(collision_world=_world())
    detached = _payload_object(attached_to_link=None)
    with pytest.raises(ValueError, match="附着"):
        _payload_request(collision_world=_world(detached))
    # 位置对了但几何不对：不能声称规划器看到了这个载荷。
    boxed = _payload_object(
        geometry=(CollisionGeometry("box", size_m=(0.05, 0.05, 0.05)),)
    )
    with pytest.raises(ValueError, match="几何"):
        _payload_request(collision_world=_world(boxed))
    # 少了圆柱也不行。
    with pytest.raises(ValueError, match="几何|碰撞"):
        _payload_request(collision_world=_world(_payload_object(geometry=(
            CollisionGeometry("cylinder", radius_m=0.01, length_m=0.01),
        ))))


def test_unrepresentable_payload_profiles_are_rejected_explicitly() -> None:
    """球/锥、带局部位姿的基元、只有资产引用的 Profile 都不能静默进规划世界。"""

    sphere = _profile(
        collision_primitives=(
            CollisionPrimitive("sphere", (0.02,), RigidTransform.identity()),
        )
    )
    with pytest.raises(ValueError, match="规划|不支持|基元"):
        PayloadAttachment(_attachment(), sphere)
    offset = _profile(
        collision_primitives=(
            CollisionPrimitive(
                "cylinder",
                (0.118, 0.0483),
                RigidTransform((0.0, 0.0, 0.059), (0.0, 0.0, 0.0, 1.0)),
            ),
        )
    )
    with pytest.raises(ValueError, match="局部|位姿|对齐"):
        PayloadAttachment(_attachment(), offset)
    asset_only = _profile(
        collision_primitives=(),
        collision_asset_ref="package://beaker/collision.stl",
    )
    with pytest.raises(ValueError, match="基元|碰撞"):
        PayloadAttachment(_attachment(), asset_only)


def test_payload_must_be_declared_and_carried() -> None:
    """payload 的 AttachmentSpec 必须出现在 attachments/carried_body_ids 中。"""

    with pytest.raises(ValueError, match="attachments"):
        _payload_request(attachments=())
    with pytest.raises(ValueError, match="carried_body_ids"):
        _payload_request(carried_body_ids=())
    with pytest.raises(ValueError, match="法兰|flange"):
        _payload_request(
            collision_world=_world(_payload_object(attached_to_link="link5")),
            attachments=(_attachment(link_name="link5"),),
            payload=_payload(attachment=_attachment(link_name="link5")),
        )
    with pytest.raises(ValueError, match="held_material"):
        _payload_request(held_material="other-material@v1")


# ------------------------------------------------------------------ 结果回显


def test_result_echoes_payload_and_rejects_missing_or_mismatched_identity() -> None:
    """结果必须回显 identity/profile/attachment digest；缺失或错配都拒绝。"""

    request = _payload_request()
    echoed = _result(request, **request.payload_fields)
    assert echoed.payload_ref == PROFILE_REF
    assert echoed.payload_profile_digest == PROFILE_DIGEST
    assert echoed.attachment_digest == request.payload.digest
    assert echoed.as_mapping()["payload"]["attachment_digest"] == request.payload.digest
    asserted = _result(request)
    assert asserted.digest != echoed.digest
    echoed.validate_payload(request)
    echoed.validate_against(request)

    with pytest.raises(ValueError, match="未记录"):
        _result(request).validate_payload(request)
    with pytest.raises(ValueError, match="identity"):
        _result(
            request,
            **{**request.payload_fields, "payload_ref": "other@v1"},
        ).validate_payload(request)
    with pytest.raises(ValueError, match="profile"):
        _result(
            request,
            **{**request.payload_fields, "payload_profile_digest": "d" * 64},
        ).validate_payload(request)
    with pytest.raises(ValueError, match="attachment"):
        _result(
            request,
            **{**request.payload_fields, "attachment_digest": "e" * 64},
        ).validate_payload(request)
    # 请求没有 payload，结果却带着 payload：不允许。
    with pytest.raises(ValueError, match="未声明"):
        _result(_request(), **request.payload_fields).validate_payload(_request())
    # 只回显一部分（否则等于伪造身份）在构造期就拒绝。
    with pytest.raises(ValueError, match="同时|payload"):
        PlanResult(
            status="success",
            waypoints=(PlanWaypoint((0.0,) * 6, 0.0, 0.0),),
            planner=_identity(),
            payload_ref=PROFILE_REF,
        )


# ------------------------------------------------------------------ 关节连续


def _chained(
    start: tuple[float, ...], end: tuple[float, ...], *, times: tuple[float, float] = (0.0, 1.0)
) -> PlanResult:
    return PlanResult(
        status="success",
        waypoints=(
            PlanWaypoint(start, 0.0, times[0]),
            PlanWaypoint(end, 0.0, times[1]),
        ),
        planner=_identity(),
    )


def test_joint_continuity_accepts_an_exactly_chained_chain() -> None:
    """相邻段端点逐位相接的链是连续的（max gap 为 0）。"""

    q0 = (0.0,) * 6
    q1 = (0.2,) * 6
    q2 = (0.4,) * 6
    report = check_joint_continuity(
        [_chained(q0, q1), _chained(q1, q2)]
    )

    assert report.ok is True
    assert report.violations == ()
    assert report.max_segment_gap_rad == 0.0
    assert report.segment_count == 2
    assert report.as_mapping()["ok"] is True


def test_joint_continuity_rejects_two_pi_relabel_and_plain_gap() -> None:
    """6.28 rad 的整圈重标定与普通端点差都必须让链无效。"""

    q0 = (0.0,) * 6
    q1 = (0.2, 0.2, 0.2, -1.009353303, 1.570796327, 1.628122894)
    relabel = tuple(
        value + (2.0 * math.pi if index == 3 else 0.0)
        for index, value in enumerate(q1)
    )
    report = check_joint_continuity([_chained(q0, q1), _chained(relabel, q0)])

    assert report.ok is False
    assert report.violations[0]["kind"] == "two_pi_jump"
    assert report.violations[0]["segment_index"] == 1
    assert report.violations[0]["joint_index"] == 3
    assert report.max_segment_gap_rad == pytest.approx(2.0 * math.pi, abs=1e-6)

    gap = tuple(value + (0.4 if index == 3 else 0.0) for index, value in enumerate(q1))
    plain = check_joint_continuity([_chained(q0, q1), _chained(gap, q0)])
    assert plain.ok is False
    assert plain.violations[0]["kind"] == "segment_gap"
    assert plain.violations[0]["gap_rad"] == pytest.approx(0.4)


def test_joint_continuity_rejects_in_segment_two_pi_jump_and_empty_trail() -> None:
    """段内单步 2π 也是一个整圈跳变；没有轨迹的段不能被静默跳过。"""

    q0 = (0.0,) * 6
    q1 = (0.3,) * 6
    wrapped = tuple(
        value + (2.0 * math.pi if index == 1 else 0.0)
        for index, value in enumerate(q0)
    )
    jumping = PlanResult(
        status="success",
        waypoints=(
            PlanWaypoint(q0, 0.0, 0.0),
            PlanWaypoint(wrapped, 0.0, 1.0),
            PlanWaypoint(q1, 0.0, 2.0),
        ),
        planner=_identity(),
    )
    report = check_joint_continuity([jumping])
    assert report.ok is False
    assert {item["kind"] for item in report.violations} == {"two_pi_jump"}
    assert report.violations[0]["scope"] == "waypoint"
    assert report.violations[0]["joint_index"] == 1

    failed = PlanResult(status="collision", waypoints=(), planner=_identity())
    missing = check_joint_continuity([_chained(q0, q1), failed])
    assert missing.ok is False
    assert missing.violations[0]["kind"] == "missing_trajectory"
    assert missing.violations[0]["segment_index"] == 1


def test_continuity_limits_are_finite_and_non_negative() -> None:
    """连续性阈值本身必须是有限非负数；不连续不允许用负阈值放行。"""

    assert JointContinuityLimits().max_segment_gap_rad == 1e-6
    for value in (-1e-3, float("nan"), float("inf")):
        with pytest.raises((TypeError, ValueError)):
            JointContinuityLimits(max_segment_gap_rad=value)
    limits = JointContinuityLimits(max_segment_gap_rad=0.5)
    assert limits.max_segment_gap_rad == 0.5
