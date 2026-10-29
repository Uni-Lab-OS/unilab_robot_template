"""快换夹爪和夹持物料附着合同的冻结决策测试。"""

from __future__ import annotations

import time

import pytest
from unilab_robot_contracts import (
    AttachmentEvidence,
    AttachmentState,
    CollisionPrimitive,
    GripperObservation,
    ObservationState,
    PayloadCollisionProfile,
    RigidTransform,
    ToolAttachmentObservation,
    ToolContextActivationReceipt,
    ToolDefinition,
)
from unilab_robot_runtime import AttachmentProjector


def _tool() -> ToolDefinition:
    """返回带静态保守碰撞包络和稳定 grasp frame 的工具。"""

    return ToolDefinition(
        tool_ref="parallel-gripper",
        model_digest="a" * 64,
        mount_to_tcp=RigidTransform(
            (0.0, 0.0, 0.18),
            (0.0, 0.0, 0.0, 1.0),
        ),
        collision_asset_ref="package://parallel_gripper/collision.stl",
        visual_asset_ref="package://parallel_gripper/visual.urdf",
        collision_digest="b" * 64,
        flange_frame="tool0",
        tcp_frame="parallel_gripper_tcp",
        grasp_frame="parallel_gripper_grasp",
        mass_kg=1.2,
        center_of_mass_m=(0.0, 0.0, 0.08),
        static_collision_primitives=(
            CollisionPrimitive(
                "box",
                (0.12, 0.10, 0.22),
                RigidTransform(
                    (0.0, 0.0, 0.11),
                    (0.0, 0.0, 0.0, 1.0),
                ),
            ),
        ),
    )


def _payload() -> PayloadCollisionProfile:
    """返回不依赖视觉 URDF 的物料碰撞 Profile。"""

    return PayloadCollisionProfile(
        profile_ref="beaker-100ml@1",
        digest="c" * 64,
        mass_kg=0.2,
        center_of_mass_m=(0.0, 0.0, 0.04),
        root_to_grasp=RigidTransform.identity(),
        collision_primitives=(
            CollisionPrimitive(
                "cylinder",
                (0.10, 0.035),
                RigidTransform(
                    (0.0, 0.0, 0.05),
                    (0.0, 0.0, 0.0, 1.0),
                ),
            ),
        ),
    )


def test_tool_projection_requires_four_matching_witnesses() -> None:
    """工具身份、锁紧、代次和 PlanningScene 回读必须全部一致。"""

    now = time.time()
    tool = _tool()
    context = tool.context(7)
    projector = AttachmentProjector(source="robot-runtime", source_boot_id="boot-1")
    observation = ToolAttachmentObservation(
        ObservationState.KNOWN,
        now,
        1.0,
        "tool-sensor",
        tool.tool_ref,
        7,
        True,
    )
    receipt = ToolContextActivationReceipt(
        True,
        context.digest,
        7,
        now,
        "moveit-readback",
    )

    projection = projector.project_tool_attached(
        robot_ref="robot-main",
        definition=tool,
        observation=observation,
        context=context,
        activation_receipt=receipt,
        now=now,
    )

    assert projection.state is AttachmentState.ATTACHED
    assert projection.anchor.link_name == "tool0"
    assert projection.as_dict()["local_pose"]["xyz_m"] == [0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="PlanningScene"):
        projector.project_tool_attached(
            robot_ref="robot-main",
            definition=tool,
            observation=observation,
            context=context,
            activation_receipt=ToolContextActivationReceipt(
                True,
                "d" * 64,
                7,
                now,
                "moveit-readback",
            ),
            now=now,
        )


def test_payload_projection_uses_grasp_frame_and_tiered_evidence() -> None:
    """弱证据只能标记 controller_confirmed，且仍使用显式 grasp frame。"""

    now = time.time()
    projector = AttachmentProjector(source="robot-runtime", source_boot_id="boot-1")
    unknown = GripperObservation(
        ObservationState.UNKNOWN,
        now,
        1.0,
        "plc-no-payload-sensor",
        None,
        None,
    )

    projection = projector.project_payload(
        payload_ref="material:beaker-42",
        tool=_tool(),
        profile=_payload(),
        local_pose=RigidTransform(
            (0.0, 0.0, -0.05),
            (0.0, 0.0, 0.0, 1.0),
        ),
        observation=unknown,
        attachment_generation=9,
        expected_holding=True,
        exact_completion_generation=9,
        exact_completion_command_ref="pick-42",
        exact_completion_observed_at=now,
        command_ref="pick-42",
        now=now,
    )

    assert projection.state is AttachmentState.ATTACHED
    assert projection.evidence is AttachmentEvidence.CONTROLLER_CONFIRMED
    assert projection.anchor.link_name == "parallel_gripper_grasp"
    assert projection.context_digest == _payload().digest


def test_weak_payload_evidence_requires_exact_fresh_command_receipt() -> None:
    """同代次的旧命令回执不得被下一次抓取复用。"""

    now = time.time()
    projection = AttachmentProjector(
        source="robot-runtime",
        source_boot_id="boot-1",
    ).project_payload(
        payload_ref="material:beaker-42",
        tool=_tool(),
        profile=_payload(),
        local_pose=RigidTransform.identity(),
        observation=GripperObservation(
            ObservationState.UNKNOWN,
            now,
            1.0,
            "plc-no-payload-sensor",
            None,
            None,
        ),
        attachment_generation=9,
        expected_holding=True,
        exact_completion_generation=9,
        exact_completion_command_ref="old-pick",
        exact_completion_observed_at=now,
        command_ref="new-pick",
        now=now,
    )

    assert projection.state is AttachmentState.UNCERTAIN
    assert projection.evidence is AttachmentEvidence.NONE


def test_payload_without_sensor_or_exact_generation_stays_uncertain() -> None:
    """普通成功返回不得伪造物料已夹取。"""

    now = time.time()
    projection = AttachmentProjector(
        source="robot-runtime",
        source_boot_id="boot-1",
    ).project_payload(
        payload_ref="material:beaker-42",
        tool=_tool(),
        profile=_payload(),
        local_pose=RigidTransform.identity(),
        observation=GripperObservation(
            ObservationState.UNKNOWN,
            now,
            1.0,
            "unknown",
            None,
            None,
        ),
        attachment_generation=2,
        expected_holding=True,
        now=now,
    )

    assert projection.state is AttachmentState.UNCERTAIN
    assert projection.evidence is AttachmentEvidence.NONE


def test_payload_profile_rejects_missing_collision_geometry() -> None:
    """视觉模型引用不能替代经批准的 PayloadCollisionProfile。"""

    with pytest.raises(ValueError, match="碰撞资产或保守基元"):
        PayloadCollisionProfile(
            profile_ref="bad@1",
            digest="e" * 64,
            mass_kg=0.1,
            center_of_mass_m=(0.0, 0.0, 0.0),
            root_to_grasp=RigidTransform.identity(),
        )


def test_tool_definition_defaults_grasp_transform_to_tcp() -> None:
    """静态夹爪仍须给 MoveIt 一个可组合的法兰到抓取 frame 变换。"""

    definition = _tool()

    assert definition.mount_to_grasp == definition.mount_to_tcp
    assert definition.context(2).context_id == definition.tool_ref
