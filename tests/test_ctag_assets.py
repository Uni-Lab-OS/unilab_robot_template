"""冻结 CTAG 型号来源、可搬移性与旧派生结果的合同。"""
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

import pytest

from unilab_end_effector_sim import ctag_assets
from unilab_end_effector_sim.ctag_reference import pad_bounds, projected_pad_gap
from unilab_end_effector_sim.urdf_reference import MimicModel


def _tree_value(node):
    return [node.tag, sorted(node.attrib.items()), [_tree_value(child) for child in node]]


def test_model_matches_frozen_ctag_subtree(tmp_path):
    path = ctag_assets.prepare_ctag_model(tmp_path / "model", max_velocity_rad_s=0.5)
    tree = ET.parse(path).getroot()
    children = [_tree_value(node) for node in tree]
    digest = hashlib.sha256(json.dumps(children, separators=(",", ":")).encode()).hexdigest()
    assert digest == "9a9287479b5ca4f657ac296c7961d5d2d626ce3190a8db8316422dffc1656b35"
    model = MimicModel(path)
    assert model.root == "ctag_base_link"
    assert model.independent_names == ["ctag_Left_1_Joint"]
    assert len(model.names) == 8
    bounds = pad_bounds(model)
    assert projected_pad_gap(bounds, model.nominal_fk({"ctag_Left_1_Joint": 0.2})) == pytest.approx(0.0055813615868799535)
    assert projected_pad_gap(bounds, model.nominal_fk({"ctag_Left_1_Joint": 0.9})) == pytest.approx(0.06592023513288456)
    assert projected_pad_gap(bounds, model.nominal_fk({"ctag_Left_1_Joint": 0.0})) < 0


def test_asset_can_move_without_original_workspace(tmp_path):
    original = tmp_path / "original"
    ctag_assets.prepare_ctag_model(original, max_velocity_rad_s=0.5)
    moved = tmp_path / "moved"
    original.rename(moved)
    for row in json.loads((moved / "bundle.json").read_text())["artifacts"]:
        assert hashlib.sha256((moved / row["path"]).read_bytes()).hexdigest() == row["sha256"]
    model = MimicModel(moved / "model.urdf")
    assert len(pad_bounds(model)) == 2
    assert not any(str(original) in p.read_text() for p in moved.glob("*.json"))


@pytest.mark.parametrize("velocity", [0, -1, float("nan"), float("inf"), True, "0.5"])
def test_invalid_development_velocity_creates_nothing(tmp_path, velocity):
    output = tmp_path / "out"
    with pytest.raises(ValueError):
        ctag_assets.prepare_ctag_model(output, max_velocity_rad_s=velocity)
    assert not output.exists()


@pytest.mark.parametrize("relative", ["source-lock.json", "source/portable.urdf", "meshes/Left_Pad_Link.STL"])
def test_changed_source_creates_nothing(tmp_path, monkeypatch, relative):
    copied = tmp_path / "sources"
    shutil.copytree(ctag_assets._ASSET_ROOT, copied)
    target = copied / relative
    target.write_bytes(target.read_bytes() + b"changed")
    monkeypatch.setattr(ctag_assets, "_ASSET_ROOT", copied)
    with pytest.raises(ValueError):
        ctag_assets.prepare_ctag_model(tmp_path / "out", max_velocity_rad_s=0.5)
    assert not (tmp_path / "out").exists()


def test_existing_output_is_preserved(tmp_path):
    output = tmp_path / "out"
    output.mkdir()
    marker = output / "keep"
    marker.write_text("original")
    with pytest.raises(FileExistsError):
        ctag_assets.prepare_ctag_model(output, max_velocity_rad_s=0.5)
    assert list(output.iterdir()) == [marker]
    assert marker.read_text() == "original"


def test_explicit_velocity_changes_only_declared_joint_limits(tmp_path):
    first = ctag_assets.prepare_ctag_model(tmp_path / "first", max_velocity_rad_s=0.5)
    second = ctag_assets.prepare_ctag_model(tmp_path / "second", max_velocity_rad_s=0.25)
    a, b = ET.parse(first).getroot(), ET.parse(second).getroot()
    for left, right in zip(a.findall("joint/limit"), b.findall("joint/limit")):
        assert float(left.get("velocity")) == 2 * float(right.get("velocity"))
        left.attrib.pop("velocity")
        right.attrib.pop("velocity")
    assert _tree_value(a) == _tree_value(b)
    record = json.loads((second.parent / "mechanics.json").read_text())
    assert record["physics_qualified"] is record["contact_qualified"] is record["hardware_qualified"] is False
