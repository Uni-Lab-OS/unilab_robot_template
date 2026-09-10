"""MODEL_DESCRIPTOR.model_ref 与 PointSet 对齐规则。"""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from _model_facts import (  # noqa: E402
    expected_domain_model_ref,
    patch_moveit_model_descriptor_ref,
    read_moveit_model_descriptor_ref,
)


def test_expected_domain_model_ref() -> None:
    assert (
        expected_domain_model_ref("szlab_poly_studio", "szlab_mixer_robot")
        == "package://szlab_poly_studio/devices/szlab_mixer_robot/models/model.yaml"
    )


def test_patch_moveit_model_descriptor_ref_replaces_ros_share_prefix() -> None:
    original = '''MODEL_DESCRIPTOR = ArmModelDescriptor(
    model_ref=(
        "package://szlab/szlab_poly_studio/devices/szlab_mixer_robot/models/model.yaml"
    ),
    planning_group="szlab_mixer_robot_arm",
)
'''
    expected = expected_domain_model_ref("szlab_poly_studio", "szlab_mixer_robot")
    updated, changed = patch_moveit_model_descriptor_ref(original, expected)
    assert changed is True
    assert expected in updated
    assert "package://szlab/szlab_poly_studio" not in updated


def test_patch_moveit_model_descriptor_ref_idempotent() -> None:
    expected = expected_domain_model_ref("szlab_poly_studio", "szlab_mixer_robot")
    original = f'MODEL_DESCRIPTOR = ArmModelDescriptor(\n    model_ref="{expected}",\n)\n'
    updated, changed = patch_moveit_model_descriptor_ref(original, expected)
    assert changed is False
    assert updated == original
