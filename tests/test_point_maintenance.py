"""PointSet 维护生命周期的公共服务测试。"""

from __future__ import annotations

from pathlib import Path

from unilab_arm_cr7 import MODEL_DESCRIPTOR
from unilab_robot_contracts import (
    CommandResult,
    CommandState,
    RigidTransform,
    ToolContext,
)
from unilab_robot_runtime import PointMaintenanceService


class SuccessfulMaintenanceSession:
    """记录低速点位试运行并返回确定完成见证。"""

    def __init__(self) -> None:
        self.commands: list[object] = []

    @property
    def target_revision(self) -> str:
        return "cr7-demo@1.0.0"

    def execute(self, command: object) -> CommandResult:
        self.commands.append(command)
        return CommandResult(command.command_id, CommandState.SUCCEEDED, "done")  # type: ignore[attr-defined]


def test_point_set_requires_all_targets_tested_before_immutable_publish(
    tmp_path: Path,
) -> None:
    """草稿必须解析、逐点低速完成并资格确认后才可发布。"""

    draft = tmp_path / "points.yaml"
    draft.write_text(
        """schema: unilab.arm-point-set/v2
revision: cr7-demo@1.0.0
compatible_model_ref: package://unilab_arm_cr7/models/model.yaml
tool_context_ref: tool-demo
targets:
  position1:
    waypoints:
      approach:
        type: joint_positions
        value: [0.0, -0.2, 0.3, 0.0, 0.2, 0.0]
""",
        encoding="utf-8",
    )
    service = PointMaintenanceService(
        model=MODEL_DESCRIPTOR,
        tool_context=ToolContext(
            "tool-demo",
            "a" * 64,
            RigidTransform.identity(),
            1,
        ),
        qualification_root=tmp_path / "qualification",
        publication_root=tmp_path / "published",
    )
    validated = service.validate_draft(draft)
    session = SuccessfulMaintenanceSession()

    service.test_target(
        validated,
        "position1.approach",
        session=session,
        command_id="point-test-1",
        hardware_profile_digest="profile-digest",
        source_boot_id="boot-1",
        monotonic_sequence=1,
    )
    qualification = service.qualify(validated, approved_by="operator-a")
    published = service.publish(validated, qualification)

    assert validated.target_refs == ("position1.approach",)
    assert published.path.is_file()
    assert published.digest == validated.digest
    assert published.path.read_bytes() == draft.read_bytes()
    assert qualification.evidence_command_ids == (
        ("position1.approach", "point-test-1"),
    )
    assert (
        tmp_path
        / "qualification"
        / "evidence"
        / validated.digest
        / "position1.approach.json"
    ).is_file()
