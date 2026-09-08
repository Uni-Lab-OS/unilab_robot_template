from __future__ import annotations

from unilab_robot_model_kit import (
    is_allowed_arm_provider,
    is_catalog_arm_provider,
    is_domain_arm_provider,
)
from unilab_arm_cr7 import build_moveit_model as build_cr7_moveit_model


def test_catalog_and_domain_arm_providers() -> None:
    assert is_catalog_arm_provider("unilab_arm_cr7:build_moveit_model")
    assert is_domain_arm_provider("my_lab_arm:build_moveit_model")
    assert is_allowed_arm_provider("unilab_arm_cr5:build_moveit_model")
    assert is_allowed_arm_provider("szlab_poly_studio.my_arm:build_moveit_model")
    assert not is_allowed_arm_provider("unilab_arm_cr7.moveit_model:build_moveit_model")


def test_cr7_still_builds_through_model_kit_pipeline() -> None:
    bundle = build_cr7_moveit_model(device_id="robot_main")
    assert len(bundle.qualified_joint_names) == 6
    assert "robot_main_cr7_tool0" in bundle.execution_urdf
    assert "mock_components/GenericSystem" in bundle.execution_urdf
    assert bundle.rviz_required is False
