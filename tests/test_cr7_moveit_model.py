"""CR7 distribution 自包含 MoveIt 模型的公开合同。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from unilab_arm_cr7 import MODEL_DESCRIPTOR
from unilab_arm_cr7.moveit_model import (
    build_joint_state_name_map,
    build_moveit_model,
)
from unilab_robot_contracts import RigidTransform, ToolContext


def test_cr7_moveit_model_is_six_axis_and_headless() -> None:
    """型号包必须生成六轴运动模型，且不包含导轨或 RViz 进程。"""

    bundle = build_moveit_model(
        device_id="robot_a",
        position={"x": 500, "y": 1500, "z": 0},
        rotation={"x": 0, "y": 0, "z": 1.57},
    )
    urdf = ET.fromstring(bundle.urdf)
    srdf = ET.fromstring(bundle.srdf)
    movable_joints = tuple(
        joint.attrib["name"]
        for joint in urdf.findall("joint")
        if joint.attrib.get("type") != "fixed"
    )

    assert movable_joints == tuple(
        f"robot_a_cr7_joint_{index}" for index in range(1, 7)
    )
    assert "arm_base_joint" not in bundle.urdf
    assert "rail" not in bundle.urdf.lower()
    assert "rviz" not in bundle.urdf.lower()
    assert srdf.find("group").attrib["name"] == "robot_a_cr7_arm"
    assert tuple(bundle.kinematics) == ("robot_a_cr7_arm",)
    assert bundle.rviz_required is False
    assert "world_mount_joint" in bundle.execution_urdf
    assert "world_mount_joint" not in bundle.render_urdf
    assert "ros2_control" not in bundle.render_urdf
    assert "robot_a/meshes/base_link0.STL" in bundle.render_urdf
    assert "file://" not in bundle.render_urdf
    assert "file://" in bundle.execution_urdf
    assert len(bundle.topology_digest) == 64


def test_cr7_moveit_model_qualifies_controller_and_joint_names() -> None:
    """两台机械臂的 ROS 名称必须由 Device id 隔离，避免控制器冲突。"""

    first = build_moveit_model(device_id="robot_a")
    second = build_moveit_model(device_id="robot_b")
    first_names = first.moveit_controllers["moveit_simple_controller_manager"][
        "controller_names"
    ]
    second_names = second.moveit_controllers["moveit_simple_controller_manager"][
        "controller_names"
    ]

    assert first_names == ["robot_a_cr7_controller"]
    assert second_names == ["robot_b_cr7_controller"]
    assert first_names != second_names
    assert set(first.joint_limits["joint_limits"]) == {
        f"robot_a_cr7_joint_{index}" for index in range(1, 7)
    }
    assert first.qualified_joint_names == tuple(
        f"robot_a_cr7_joint_{index}" for index in range(1, 7)
    )
    assert first.topology_digest != second.topology_digest


def test_cr7_joint_feedback_requires_exact_complete_mapping() -> None:
    """SDK/PLC 只有在型号包声明 exact 映射后才能成为观测。"""

    mapping = build_joint_state_name_map(device_id="robot_b")

    assert tuple(mapping.qualify(mapping.canonical_joint_names, (0.0,) * 6)) == (
        "robot_b_cr7_joint_1",
        "robot_b_cr7_joint_2",
        "robot_b_cr7_joint_3",
        "robot_b_cr7_joint_4",
        "robot_b_cr7_joint_5",
        "robot_b_cr7_joint_6",
    )
    with pytest.raises(ValueError, match="未验证 joint-state source"):
        build_joint_state_name_map(
            device_id="robot_b",
            source="unverified_plc_register_order",
        )


def test_cr7_moveit_model_owns_exact_source_and_mesh_assets() -> None:
    """安装后的 distribution 必须自带已锁定 URDF 和全部 CR7 mesh。"""

    bundle = build_moveit_model(device_id="robot_a")

    assert bundle.source_digest == (
        "c64ced0cdcb2654dc07190dc0b9c4d3db813a9515507a69c5c027d82547c14c7"
    )
    assert len(bundle.mesh_paths) == 7
    assert all(path.is_file() for path in bundle.mesh_paths)
    assert "package://dobot_rviz" not in bundle.urdf


def test_cr7_descriptor_owns_joint_units_limits_and_forward_kinematics() -> None:
    """点位文件删除 joint_names 后，型号描述符必须拥有顺序、限位和 FK。"""

    tool_context = ToolContext(
        context_id="cr7-default-tool@1.0.0",
        digest="0" * 64,
        mount_to_tcp=RigidTransform.identity(),
        attachment_generation=1,
    )

    pose = MODEL_DESCRIPTOR.forward_kinematics((0.0,) * 6, tool_context)

    assert MODEL_DESCRIPTOR.joint_names == tuple(
        f"cr7_joint_{index}" for index in range(1, 7)
    )
    assert tuple(
        specification.joint_type.canonical_unit
        for specification in MODEL_DESCRIPTOR.joint_specs
    ) == ("rad",) * 6
    assert MODEL_DESCRIPTOR.joint_specs[2].lower == pytest.approx(-2.79)
    assert pose.frame_ref == "arm_base"
    assert pose.xyz_m == pytest.approx((0.0, -0.246, 0.947), abs=1e-5)
