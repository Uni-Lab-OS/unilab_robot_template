"""导轨型号包的关节命名必须与机械臂使用同一套完全限定规则。"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from unilab_rail_linear import (
    MODEL_DESCRIPTOR,
    build_joint_state_name_map,
    build_kinematic_model,
)
from unilab_rail_linear.adapters.simulation import SimulationRailAxisPort
from unilab_rail_linear.factory import create_simulation_module


def test_rail_joint_names_are_device_id_qualified() -> None:
    """两台导轨的 /joint_states 名称必须由 Graph device_id 隔离。"""

    first = build_joint_state_name_map(device_id="rail")
    second = build_joint_state_name_map(device_id="rail_b")

    assert first.canonical_joint_names == (MODEL_DESCRIPTOR.axis_joint,)
    assert first.qualified_joint_names == ("rail_rail_joint",)
    assert second.qualified_joint_names == ("rail_b_rail_joint",)
    assert first.qualify(("rail_joint",), (0.35,)) == {
        "rail_rail_joint": pytest.approx(0.35)
    }


def test_rail_kinematic_model_declares_one_prismatic_joint() -> None:
    """渲染 URDF 只声明导轨自己的一根轴，不并入机械臂关节。"""

    first = build_kinematic_model(device_id="rail")
    second = build_kinematic_model(device_id="rail_b")
    urdf = ET.fromstring(first.render_urdf)
    movable = tuple(
        joint.attrib["name"]
        for joint in urdf.findall("joint")
        if joint.attrib.get("type") != "fixed"
    )

    assert movable == ("rail_rail_joint",)
    assert first.qualified_joint_names == ("rail_rail_joint",)
    assert first.mount_link == "rail_rail_carriage"
    assert "cr5_joint" not in first.render_urdf
    assert first.topology_digest != second.topology_digest
    assert len(first.topology_digest) == 64
    assert first.source_digest == (
        "9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d"
    )


def test_rail_carriage_is_a_visual_free_mount_link() -> None:
    """滑座只提供机械臂挂载 frame，导轨外观由领域静态 mesh 唯一渲染。"""

    urdf = ET.fromstring(build_kinematic_model(device_id="rail").render_urdf)
    carriage = next(
        link
        for link in urdf.findall("link")
        if link.attrib["name"] == "rail_rail_carriage"
    )
    assert carriage.find("visual") is None
    assert carriage.find("collision") is None


def test_simulation_module_moves_to_declared_station_si() -> None:
    """仿真端口必须把位置设成部署目标 SI，而不是每次累加。"""

    module = create_simulation_module(
        endpoint_ids=frozenset({"sim:rail"}),
        target_data={"targets": {"collect": 0.35, "tool-change": 0.5}},
        on_settled=lambda _command_id: None,
    )
    observed = module.move_and_settle("cmd-1", "collect")

    assert observed.position == pytest.approx(0.35)
    assert observed.target_ref == "collect"
    assert observed.settled is True


def test_simulation_port_without_targets_keeps_legacy_increment() -> None:
    """未注入目标集时保留原互锁测试使用的累加语义。"""

    port = SimulationRailAxisPort(endpoint_id="rail:a")
    port.move("cmd-1", "S04")
    assert port.observe().position == pytest.approx(1.0)
