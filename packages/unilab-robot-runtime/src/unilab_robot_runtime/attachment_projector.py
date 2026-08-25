"""从真实快换/夹爪证据生成 format-free 运动学附着 latest。"""

from __future__ import annotations

import math
import time

from unilab_robot_contracts import (
    AnchorKind,
    AttachmentEvidence,
    AttachmentKind,
    AttachmentState,
    GripperObservation,
    KinematicAnchor,
    KinematicAttachmentProjection,
    ObservationState,
    PayloadCollisionProfile,
    RigidTransform,
    ToolAttachmentObservation,
    ToolContext,
    ToolContextActivationReceipt,
    ToolDefinition,
)


class AttachmentProjector:
    """集中校验证据并生成 OS/FE 可消费的附着投影。"""

    def __init__(self, *, source: str, source_boot_id: str) -> None:
        """绑定投影器来源和运行代际。"""

        if not source.strip() or not source_boot_id.strip():
            raise ValueError("AttachmentProjector 必须绑定来源和 boot_id")
        self.source = source
        self.source_boot_id = source_boot_id
        self._sequence = 0

    def project_tool_attached(
        self,
        *,
        robot_ref: str,
        definition: ToolDefinition,
        observation: ToolAttachmentObservation,
        context: ToolContext,
        activation_receipt: ToolContextActivationReceipt,
        command_ref: str | None = None,
        job_ref: str | None = None,
        now: float | None = None,
    ) -> KinematicAttachmentProjection:
        """在四项证据一致后把工具根节点挂到机械臂法兰。"""

        checked_at = time.time() if now is None else float(now)
        known = (
            observation.state is ObservationState.KNOWN
            and 0.0 <= checked_at - observation.observed_at <= observation.max_age_s
            and observation.tool_ref == definition.tool_ref
            and observation.locked is True
            and observation.attachment_generation == context.attachment_generation
            and context.context_id == definition.tool_ref
            and activation_receipt.applied
            and 0.0
            <= checked_at - activation_receipt.observed_at
            <= observation.max_age_s
            and activation_receipt.tool_context_digest == context.digest
            and activation_receipt.attachment_generation
            == context.attachment_generation
        )
        if not known:
            raise ValueError("工具身份、锁紧、附着代次或 PlanningScene 回读不一致")
        return self._projection(
            kind=AttachmentKind.TOOL,
            child_ref=definition.tool_ref,
            parent_ref=robot_ref,
            anchor=KinematicAnchor(AnchorKind.LINK, definition.flange_frame),
            local_pose=definition.mount_to_visual,
            state=AttachmentState.ATTACHED,
            evidence=AttachmentEvidence.OBSERVED,
            attachment_generation=context.attachment_generation,
            observed_at=max(observation.observed_at, activation_receipt.observed_at),
            stale_after_s=observation.max_age_s,
            command_ref=command_ref,
            job_ref=job_ref,
            context_digest=context.digest,
        )

    def project_payload(
        self,
        *,
        payload_ref: str,
        tool: ToolDefinition,
        profile: PayloadCollisionProfile,
        local_pose: RigidTransform,
        observation: GripperObservation,
        attachment_generation: int,
        expected_holding: bool,
        exact_completion_generation: int | None = None,
        exact_completion_command_ref: str | None = None,
        exact_completion_observed_at: float | None = None,
        command_ref: str | None = None,
        job_ref: str | None = None,
        now: float | None = None,
    ) -> KinematicAttachmentProjection:
        """按物理观测或精确控制器代次生成负载附着/脱离投影。"""

        checked_at = time.time() if now is None else float(now)
        if not math.isfinite(observation.max_age_s) or observation.max_age_s <= 0.0:
            raise ValueError("负载观测 max_age_s 必须是正有限数")
        observed = (
            observation.state is ObservationState.KNOWN
            and 0.0 <= checked_at - observation.observed_at <= observation.max_age_s
            and observation.holding_payload is expected_holding
        )
        controller_confirmed = (
            exact_completion_generation is not None
            and exact_completion_generation == attachment_generation
            and command_ref is not None
            and exact_completion_command_ref == command_ref
            and exact_completion_observed_at is not None
            and math.isfinite(exact_completion_observed_at)
            and 0.0
            <= checked_at - exact_completion_observed_at
            <= observation.max_age_s
        )
        if observed:
            evidence = AttachmentEvidence.OBSERVED
        elif controller_confirmed:
            evidence = AttachmentEvidence.CONTROLLER_CONFIRMED
        else:
            evidence = AttachmentEvidence.NONE
        state = (
            AttachmentState.ATTACHED
            if expected_holding
            else AttachmentState.DETACHED
        )
        if evidence is AttachmentEvidence.NONE:
            state = AttachmentState.UNCERTAIN
        evidence_time = (
            observation.observed_at
            if observed
            else exact_completion_observed_at
            if controller_confirmed
            else checked_at
        )
        if evidence_time is None:
            raise RuntimeError("负载附着证据时间缺失")
        return self._projection(
            kind=AttachmentKind.MATERIAL_PAYLOAD,
            child_ref=payload_ref,
            parent_ref=tool.tool_ref,
            anchor=KinematicAnchor(AnchorKind.LINK, tool.grasp_frame),
            local_pose=local_pose,
            state=state,
            evidence=evidence,
            attachment_generation=attachment_generation,
            observed_at=evidence_time,
            stale_after_s=observation.max_age_s,
            command_ref=command_ref,
            job_ref=job_ref,
            context_digest=profile.digest,
        )

    def _projection(
        self,
        *,
        kind: AttachmentKind,
        child_ref: str,
        parent_ref: str,
        anchor: KinematicAnchor,
        local_pose: RigidTransform,
        state: AttachmentState,
        evidence: AttachmentEvidence,
        attachment_generation: int,
        observed_at: float,
        stale_after_s: float,
        command_ref: str | None,
        job_ref: str | None,
        context_digest: str,
    ) -> KinematicAttachmentProjection:
        """分配同 boot_id 内严格单调的投影序列。"""

        self._sequence += 1
        return KinematicAttachmentProjection(
            kind=kind,
            child_ref=child_ref,
            parent_ref=parent_ref,
            anchor=anchor,
            local_pose=local_pose,
            state=state,
            evidence=evidence,
            attachment_generation=attachment_generation,
            observed_at=observed_at,
            stale_after_s=stale_after_s,
            source=self.source,
            source_boot_id=self.source_boot_id,
            monotonic_sequence=self._sequence,
            command_ref=command_ref,
            job_ref=job_ref,
            context_digest=context_digest,
        )


__all__ = ["AttachmentProjector"]
