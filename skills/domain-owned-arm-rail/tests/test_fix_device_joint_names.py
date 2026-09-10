"""fix_device_joint_names 单元测试。"""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from fix_device_joint_names import _normalize_joint_text  # noqa: E402


def test_strip_slug_from_qualified() -> None:
    original = 'names = ("szlab_mixer_robot_cr7_joint_1",)'
    updated, count = _normalize_joint_text(original, device_id="szlab_mixer_robot")
    assert count >= 1
    assert "szlab_mixer_robot_cr7_joint_1" not in updated
    assert "szlab_mixer_robot_joint_1" in updated


def test_strip_slug_from_canonical() -> None:
    original = "cr7_joint_2"
    updated, count = _normalize_joint_text(original, device_id="arm_a")
    assert updated == "joint_2"
    assert count == 1


def test_keep_device_qualified() -> None:
    original = "szlab_mixer_robot_joint_3"
    updated, count = _normalize_joint_text(original, device_id="szlab_mixer_robot")
    assert updated == original
    assert count == 0
