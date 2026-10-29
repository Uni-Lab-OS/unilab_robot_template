"""CR5/CR7/导轨型号 provider 的版本、摘要、关节顺序、限位与 TCP 合同。"""

from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from unilab_arm_cr5 import build_moveit_model as build_cr5_moveit_model
from unilab_arm_cr5.moveit_model import (
    MODEL_DESCRIPTOR_PATH as CR5_MODEL_YAML,
    MODEL_SOURCE_PATH as CR5_URDF,
    describe_model_provider as describe_cr5_provider,
)
from unilab_arm_cr7 import build_moveit_model as build_cr7_moveit_model
from unilab_arm_cr7.moveit_model import (
    MODEL_DESCRIPTOR_PATH as CR7_MODEL_YAML,
    MODEL_SOURCE_PATH as CR7_URDF,
    describe_model_provider as describe_cr7_provider,
)
from unilab_rail_linear import build_kinematic_model
from unilab_rail_linear.kinematic_model import (
    MODEL_YAML_PATH as RAIL_MODEL_YAML,
    describe_model_provider as describe_rail_provider,
)
from unilab_robot_contracts import (
    ModelProviderDescriptor,
    RigidTransform,
    model_provider_digest,
)

_DEVICE_ID = "robot_a"


def _file_digest(path: Path, *, normalize_newlines: bool = False) -> str:
    """返回 provider 摘要使用的显式文件内容 SHA-256。"""

    data = path.read_bytes()
    if normalize_newlines:
        data = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(data).hexdigest()


def _assert_arm_descriptor(
    descriptor: ModelProviderDescriptor,
    *,
    model_id: str,
    joint_prefix: str,
) -> None:
    """机械臂 descriptor 必须与型号固有六轴和单位一致。"""

    assert isinstance(descriptor, ModelProviderDescriptor)
    assert descriptor.name == model_id
    assert descriptor.version
    assert len(descriptor.digest) == 64
    assert descriptor.joint_names == tuple(
        f"{joint_prefix}_joint_{index}" for index in range(1, 7)
    )
    assert len(descriptor.joint_limits) == len(descriptor.joint_names)
    assert all(lower < upper for lower, upper in descriptor.joint_limits)
    assert descriptor.joint_units == ("rad",) * 6
    assert descriptor.tcp == RigidTransform.identity()
    assert descriptor.base_frame == "arm_base"
    assert descriptor.tip_frame == f"{joint_prefix}_link_6"
    assert descriptor.model_path is not None
    assert Path(descriptor.model_path).is_file()


def _assert_arm_matches_moveit_model(
    descriptor: ModelProviderDescriptor,
    bundle,
) -> None:
    """descriptor 的关节顺序、限位和 TCP 必须来自同一执行模型。"""

    prefix = f"{_DEVICE_ID}_"
    assert bundle.qualified_joint_names == tuple(
        f"{prefix}{name}" for name in descriptor.joint_names
    )
    urdf = ET.fromstring(bundle.execution_urdf)
    for name, (lower, upper) in zip(
        descriptor.joint_names, descriptor.joint_limits, strict=True
    ):
        joint = urdf.find(f"joint[@name='{prefix}{name}']")
        assert joint is not None
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) == pytest.approx(lower)
        assert float(limit.attrib["upper"]) == pytest.approx(upper)


def test_cr5_provider_descriptor_matches_moveit_model() -> None:
    """CR5 provider 必须按型号拥有版本、摘要、轴序、限位和 TCP。"""

    descriptor = describe_cr5_provider()
    bundle = build_cr5_moveit_model(device_id=_DEVICE_ID)

    _assert_arm_descriptor(descriptor, model_id="dobot-cr5", joint_prefix="cr5")
    _assert_arm_matches_moveit_model(descriptor, bundle)


def test_cr5_provider_digest_is_stable_and_recomputable() -> None:
    """同一显式输入的 CR5 摘要必须稳定且可独立复算。"""

    first = describe_cr5_provider()
    second = describe_cr5_provider()

    assert first == second
    assert first.digest == model_provider_digest(
        name=first.name,
        version=first.version,
        joint_names=first.joint_names,
        joint_limits=first.joint_limits,
        tcp=first.tcp,
        sources={
            "urdf": _file_digest(CR5_URDF),
            "model": _file_digest(CR5_MODEL_YAML),
        },
    )


def test_cr5_provider_digest_tracks_source_content(tmp_path: Path) -> None:
    """临时改变 CR5 URDF 内容必须改变摘要而保留同一轴序和限位。"""

    probe = tmp_path / "cr5_robot.urdf"
    original = CR5_URDF.read_bytes()
    assert b"</robot>" in original
    probe.write_bytes(
        original.replace(b"</robot>", b"<!-- provider probe -->\n</robot>")
    )

    baseline = describe_cr5_provider()
    changed = describe_cr5_provider(source_path=probe)

    assert changed.digest != baseline.digest
    assert changed.joint_names == baseline.joint_names
    assert changed.joint_limits == baseline.joint_limits


def test_cr7_provider_descriptor_matches_moveit_model() -> None:
    """CR7 provider 必须按型号拥有版本、摘要、轴序、限位和 TCP。"""

    descriptor = describe_cr7_provider()
    bundle = build_cr7_moveit_model(device_id=_DEVICE_ID)

    _assert_arm_descriptor(descriptor, model_id="dobot-cr7", joint_prefix="cr7")
    _assert_arm_matches_moveit_model(descriptor, bundle)


def test_cr7_provider_digest_is_stable_and_recomputable() -> None:
    """同一显式输入的 CR7 摘要必须稳定且可独立复算。"""

    first = describe_cr7_provider()
    second = describe_cr7_provider()

    assert first == second
    assert first.digest == model_provider_digest(
        name=first.name,
        version=first.version,
        joint_names=first.joint_names,
        joint_limits=first.joint_limits,
        tcp=first.tcp,
        sources={
            "urdf": _file_digest(CR7_URDF),
            "model": _file_digest(CR7_MODEL_YAML),
        },
    )


def test_cr7_provider_digest_tracks_source_content(tmp_path: Path) -> None:
    """临时改变 CR7 URDF 内容必须改变摘要而保留同一轴序和限位。"""

    probe = tmp_path / "cr7_robot.urdf"
    original = CR7_URDF.read_bytes()
    assert b"</robot>" in original
    probe.write_bytes(
        original.replace(b"</robot>", b"<!-- provider probe -->\n</robot>")
    )

    baseline = describe_cr7_provider()
    changed = describe_cr7_provider(source_path=probe)

    assert changed.digest != baseline.digest
    assert changed.joint_names == baseline.joint_names
    assert changed.joint_limits == baseline.joint_limits


def test_rail_provider_descriptor_matches_kinematic_model() -> None:
    """导轨 provider 必须拥有型号、单轴顺序、行程、单位和 frame。"""

    descriptor = describe_rail_provider()
    bundle = build_kinematic_model(device_id="rail")

    assert isinstance(descriptor, ModelProviderDescriptor)
    assert descriptor.name == "szlab-linear-rail-a"
    assert descriptor.version
    assert len(descriptor.digest) == 64
    assert descriptor.joint_names == ("rail_joint",)
    assert descriptor.joint_limits == ((0.0, 2.25),)
    assert descriptor.joint_units == ("m",)
    assert descriptor.tcp == RigidTransform.identity()
    assert descriptor.base_frame == "rail_base"
    assert descriptor.tip_frame == "rail_carriage"
    assert descriptor.model_path is not None
    assert Path(descriptor.model_path).is_file()

    urdf = ET.fromstring(bundle.render_urdf)
    joint = urdf.find("joint[@name='rail_rail_joint']")
    assert joint is not None
    limit = joint.find("limit")
    assert limit is not None
    assert float(limit.attrib["lower"]) == pytest.approx(descriptor.joint_limits[0][0])
    assert float(limit.attrib["upper"]) == pytest.approx(descriptor.joint_limits[0][1])
    assert limit.attrib["velocity"] == "0.25"


def test_rail_provider_digest_is_stable_and_recomputable() -> None:
    """同一 model.yaml 的导轨摘要必须稳定且可独立复算。"""

    first = describe_rail_provider()
    second = describe_rail_provider()

    assert first == second
    assert first.digest == model_provider_digest(
        name=first.name,
        version=first.version,
        joint_names=first.joint_names,
        joint_limits=first.joint_limits,
        tcp=first.tcp,
        sources={"model": _file_digest(RAIL_MODEL_YAML, normalize_newlines=True)},
    )


def test_rail_provider_digest_tracks_source_content(tmp_path: Path) -> None:
    """临时改变导轨 model.yaml 内容必须改变摘要而保留同一轴序和行程。"""

    probe = tmp_path / "model.yaml"
    probe.write_bytes(RAIL_MODEL_YAML.read_bytes() + b"# provider probe\n")

    baseline = describe_rail_provider()
    changed = describe_rail_provider(descriptor_path=probe)

    assert changed.digest != baseline.digest
    assert changed.joint_names == baseline.joint_names
    assert changed.joint_limits == baseline.joint_limits
