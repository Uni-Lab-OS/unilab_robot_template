"""关节命名规则单元测试（不依赖领域仓）。"""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from _joint_naming import (  # noqa: E402
    expected_canonical_joint_names,
    expected_qualified_joint_names,
    find_legacy_qualified_joint_literals,
    find_legacy_qualified_joint_patterns,
    is_valid_canonical_joint_name,
    joint_naming_remediation,
    validate_canonical_joint_names,
)


def test_canonical_joint_pattern() -> None:
    assert is_valid_canonical_joint_name("joint_1")
    assert is_valid_canonical_joint_name("joint_6")
    assert not is_valid_canonical_joint_name("cr7_joint_1")
    assert not is_valid_canonical_joint_name("szlab_mixer_robot_joint_1")
    assert not is_valid_canonical_joint_name("joint_0")


def test_expected_qualified_uses_device_id_only() -> None:
    qualified = expected_qualified_joint_names(device_id="arm_a", dof=6)
    assert qualified == tuple(f"arm_a_joint_{index}" for index in range(1, 7))
    assert qualified[0] != "arm_a_cr7_joint_1"


def test_validate_rejects_model_slug_in_canonical() -> None:
    errors = validate_canonical_joint_names(
        tuple(f"cr7_joint_{index}" for index in range(1, 7)),
        dof=6,
    )
    assert errors
    assert validate_canonical_joint_names(expected_canonical_joint_names(), dof=6) == []


def test_find_legacy_qualified_literals() -> None:
    sample = '''
    qualified = ("szlab_mixer_robot_cr7_joint_1", "szlab_mixer_robot_joint_2")
    '''
    hits = find_legacy_qualified_joint_literals(sample, device_id="szlab_mixer_robot")
    assert hits == ["szlab_mixer_robot_cr7_joint_"]


def test_find_legacy_fstring_patterns() -> None:
    sample = 'f"szlab_mixer_robot_cr7_joint_{index}"'
    hits = find_legacy_qualified_joint_patterns(sample, device_id="szlab_mixer_robot")
    assert "szlab_mixer_robot_cr7_joint_" in hits[0]


def test_remediation_shape() -> None:
    payload = joint_naming_remediation(device_id="robot_a", dof=6)
    assert payload["qualified_joint_names"][0] == "robot_a_joint_1"
    assert payload["moveit_model_joint_names_mapping"]["joint1"] == "joint_1"
