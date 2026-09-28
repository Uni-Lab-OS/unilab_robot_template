"""珞石CR12型号事实、模板接口及冻结参考的一致性。"""
from pathlib import Path
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET

import pytest

from unilab_arm_rokae_cr12 import MODEL_DESCRIPTOR, build_joint_state_name_map, build_moveit_model
from unilab_arm_rokae_cr12 import source_assets
from unilab_robot_contracts import RigidTransform, ToolContext
from unilab_robot_contracts.urdf_reference import Model


def _tool():
    return ToolContext(context_id="cr12-reference",digest="0"*64,mount_to_tcp=RigidTransform.identity(),attachment_generation=1)


def test_model_is_six_axis_with_separate_render_and_execution():
    bundle=build_moveit_model(device_id="arm_a",position={"x":500,"y":100,"z":200},rotation={"z":0.25})
    tree=ET.fromstring(bundle.execution_urdf)
    moving=[j for j in tree.findall("joint") if j.get("type")!="fixed"]
    assert [j.get("name") for j in moving]==[f"arm_a_rokae_cr12_joint_{i}" for i in range(1,7)]
    assert all(j.get("type")=="revolute" for j in moving)
    assert "arm_base_joint" not in bundle.execution_urdf
    assert "rail" not in bundle.execution_urdf.lower()
    assert "rviz" not in bundle.execution_urdf.lower()
    assert bundle.rviz_required is False
    assert "world_mount_joint" not in bundle.render_urdf
    assert "ros2_control" not in bundle.render_urdf
    assert tree.find("ros2_control/hardware/plugin").text=="mock_components/GenericSystem"
    mount=tree.find("joint[@name='arm_a_world_mount_joint']/origin")
    assert list(map(float,mount.get("xyz").split()))==[.5,.1,.2]
    assert list(map(float,mount.get("rpy").split()))==[0,0,.25]
    assert len(bundle.mesh_paths)==7
    assert all(p.is_file() for p in bundle.mesh_paths)
    assert "package://" not in bundle.execution_urdf
    assert "file://" in bundle.execution_urdf and "file://" not in bundle.render_urdf
    assert bundle.source_digest=="091fbc1fa23963e01c5a68f5aba7b6b1bf63b493530403371d771407d104b7ea"


def test_two_instances_have_disjoint_names():
    a,b=(build_moveit_model(device_id=n) for n in ["arm_a","arm_b"])
    for tag in ["link","joint","ros2_control"]:
        names=lambda bundle:{n.get("name") for n in ET.fromstring(bundle.execution_urdf).findall(tag)}
        assert not names(a)&names(b)
    assert a.qualified_joint_names!=b.qualified_joint_names
    assert a.topology_digest!=b.topology_digest
    assert a.moveit_controllers["moveit_simple_controller_manager"]["controller_names"]==["arm_a_rokae_cr12_controller"]
    assert b.moveit_controllers["moveit_simple_controller_manager"]["controller_names"]==["arm_b_rokae_cr12_controller"]


def test_fixed_installation_does_not_add_movable_axis():
    bundle=build_moveit_model(device_id="arm",mount_yaw_deg=180)
    tree=ET.fromstring(bundle.execution_urdf)
    joint=tree.find("joint[@name='arm_mount_yaw_joint']")
    assert joint.get("type")=="fixed"
    assert float(joint.find("origin").get("rpy").split()[2])==pytest.approx(math.pi)
    assert len(bundle.qualified_joint_names)==6
    same_topology=build_moveit_model(device_id="arm",position={"x":10})
    assert bundle.topology_digest==same_topology.topology_digest


def test_source_limits_inertias_and_collision_rules_are_preserved():
    bundle=build_moveit_model(device_id="arm")
    tree=ET.fromstring(bundle.execution_urdf)
    original=ET.fromstring(source_assets.load_source_bytes())
    for i in range(1,7):
        source=original.find(f"joint[@name='joint{i}']/limit")
        target=tree.find(f"joint[@name='arm_rokae_cr12_joint_{i}']/limit")
        assert source.attrib==target.attrib
        assert bundle.joint_limits["joint_limits"][f"arm_rokae_cr12_joint_{i}"]["max_velocity"]==float(source.get("velocity"))
        source=original.find(f"link[@name='xMateCR12_link{i}']/inertial")
        target=tree.find(f"link[@name='arm_rokae_cr12_link_{i}']/inertial")
        assert ET.tostring(source)==ET.tostring(target)
    assert float(tree.find("joint[@name='arm_rokae_cr12_joint_1']/limit").get("effort"))==436
    vendor=ET.parse(source_assets._MODEL_ROOT/"source/xMateCR12.srdf").getroot()
    mapping={"xMateCR12_base":"arm_rokae_cr12_base",**{f"xMateCR12_link{i}":f"arm_rokae_cr12_link_{i}" for i in range(1,7)}}
    expected={(mapping[n.get("link1")],mapping[n.get("link2")],n.get("reason")) for n in vendor.findall("disable_collisions")}
    actual={(n.get("link1"),n.get("link2"),n.get("reason")) for n in ET.fromstring(bundle.srdf).findall("disable_collisions")}
    assert actual==expected
    assert len(actual)==12


def test_descriptor_joint_order_units_and_zero_pose():
    assert MODEL_DESCRIPTOR.joint_names==tuple(f"rokae_cr12_joint_{i}" for i in range(1,7))
    assert tuple(j.joint_type.canonical_unit for j in MODEL_DESCRIPTOR.joint_specs)==("rad",)*6
    assert MODEL_DESCRIPTOR.joint_specs[1].lower==pytest.approx(-2.9671)
    result=MODEL_DESCRIPTOR.forward_kinematics([0]*6,_tool())
    assert result.frame_ref=="arm_base"
    assert result.xyz_m==pytest.approx([0,-.15,1.777],abs=1e-12)
    assert result.orientation_xyzw==pytest.approx([0,0,0,1])


def test_fk_matches_frozen_reference_for_64_configurations():
    reference=Model(source_assets._MODEL_ROOT/"rokae_cr12_robot.urdf")
    for index in range(64):
        joints=[.6*math.sin((index+1)*(j+1)) for j in range(6)]
        old=reference.fk(joints)["xMateCR12_link6"]
        current=MODEL_DESCRIPTOR.forward_kinematics(joints,_tool())
        transform=RigidTransform(current.xyz_m,current.orientation_xyzw)
        assert transform.translation_m==pytest.approx([old[i][3] for i in range(3)],abs=1e-12)
        # 用旋转基向量核对，避免四元数等价符号影响。
        for j in range(3):
            axis=tuple(1.0 if i==j else 0.0 for i in range(3))
            rotated=transform.compose(RigidTransform(axis,(0,0,0,1))).translation_m
            assert tuple(rotated[i]-transform.translation_m[i] for i in range(3))==pytest.approx([old[i][j] for i in range(3)],abs=1e-12)


def test_only_declared_feedback_mapping_is_accepted():
    mapping=build_joint_state_name_map(device_id="arm")
    assert len(mapping.qualify(mapping.canonical_joint_names,[0]*6))==6
    with pytest.raises(ValueError):
        build_joint_state_name_map(device_id="arm",source="dobot_sdk_v1")


@pytest.mark.parametrize("device_id",["", "with-dash", "arm/one"])
def test_invalid_instance_is_rejected(device_id):
    with pytest.raises(ValueError):build_moveit_model(device_id=device_id)


@pytest.mark.parametrize("yaw",[float("inf"),float("nan")])
def test_nonfinite_fixed_mount_is_rejected(yaw):
    with pytest.raises(ValueError):build_moveit_model(device_id="arm",mount_yaw_deg=yaw)


@pytest.mark.parametrize("relative",["source-lock.json","rokae_cr12_robot.urdf","source/xMateCR12.srdf","meshes/mesh_4320e53a37ee9be6.stl"])
def test_changed_asset_fails_before_model_creation(tmp_path,monkeypatch,relative):
    copied=tmp_path/"models"
    shutil.copytree(source_assets._MODEL_ROOT,copied)
    p=copied/relative;p.write_bytes(p.read_bytes()+b"changed")
    monkeypatch.setattr(source_assets,"_MODEL_ROOT",copied)
    with pytest.raises(ValueError):build_moveit_model(device_id="arm")


def test_no_vendor_execution_factories_are_exposed():
    import unilab_arm_rokae_cr12 as module
    assert not hasattr(module,"create_plc_backend")
    assert not hasattr(module,"create_tcp_sdk_backend")
    assert not hasattr(module,"create_moveit_backend")
