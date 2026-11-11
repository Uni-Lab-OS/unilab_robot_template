"""Brand distribution preserves existing provider identity and fails closed."""
import json
from pathlib import Path

import pytest

import unilab_robot_dobot as dobot
import unilab_robot_rokae as rokae
from unilab_robot_contracts.brand_catalog import BrandCatalog
from unilab_arm_cr5 import build_moveit_model as old_cr5


def test_brand_and_legacy_provider_are_identical():
    a = dobot.build_moveit_model("cr5", device_id="arm")
    b = old_cr5(device_id="arm")
    assert a.execution_urdf == b.execution_urdf
    assert a.srdf == b.srdf
    assert a.source_digest == b.source_digest
    assert a.qualified_joint_names == b.qualified_joint_names
    assert dobot.build_moveit_model("cr7", device_id="other").qualified_joint_names != a.qualified_joint_names


def test_module_carries_motion_composition_and_source_correspondence():
    definition = dobot.module_definition("cr5")
    assert definition["asset"] == dobot.model("cr5")["asset"]
    assert definition["composition"]["root_asset"] == definition["asset"]
    assert definition["motion_model"]["parameters"]
    assert definition["representation_correspondence"]["matching_source_joint_definitions"]
    assert definition["representation_correspondence"]["selected_usd_vs_provider"] == "not_verified"
    assert dobot.module_definition("cr10")["drivers"]["status"] == "not_implemented"


def test_model_inventory_does_not_grant_provider():
    assert "cr10" in dobot.models()
    with pytest.raises(ValueError, match="no validated provider"):
        dobot.build_moveit_model("cr10", device_id="arm")
    with pytest.raises(ValueError, match="Unknown exact model"):
        dobot.build_moveit_model("CR5_typo", device_id="arm")


def test_cr12_uses_rokae_identity_and_keeps_equivalence_unverified():
    row = rokae.model("xmatecr12")
    assert row["asset"]["asset_id"] == "f09d1a8c-7324-5c5e-875b-e53b363000a6"
    assert row["model_equivalence"] == "not_verified"
    assert len(rokae.build_moveit_model("xmatecr12", device_id="arm").qualified_joint_names) == 6


def test_profile_is_frozen_and_cannot_be_executed_as_a_model():
    profile = dobot.source_profiles()[0]
    document = dobot.catalog.source_profile(profile["id"])
    assert document["status"] == "source_preserved_not_runtime_enabled"
    assert document["files"]
    with pytest.raises(ValueError, match="Unknown exact model"):
        dobot.build_moveit_model(profile["id"], device_id="arm")


def test_source_profile_tamper_is_rejected(tmp_path):
    import hashlib
    profile = tmp_path / "profile"
    profile.mkdir()
    (profile / "config.yaml").write_text("before")
    document = {"files": [{"path": "config.yaml", "sha256": hashlib.sha256(b"before").hexdigest()}]}
    lock = profile / "lock.json"
    lock.write_text(json.dumps(document))
    (tmp_path / "catalog.json").write_text(json.dumps({"schema": "unilab.robot-brand/1", "models": {}, "source_profiles": [{"id": "p", "manifest": "profile/lock.json", "sha256": hashlib.sha256(lock.read_bytes()).hexdigest()}]}))
    catalog = BrandCatalog(str(tmp_path / "__init__.py"))
    catalog.source_profile("p")
    (profile / "config.yaml").write_text("after")
    with pytest.raises(ValueError, match="file changed"):
        catalog.source_profile("p")


def test_asset_lookup_never_falls_back_to_latest_or_upstream(tmp_path):
    with pytest.raises(FileNotFoundError):
        dobot.resolve_asset("cr5", tmp_path)


def test_asset_path_cannot_escape_explicit_cache(tmp_path):
    (tmp_path / "catalog.json").write_text(json.dumps({"schema": "unilab.robot-brand/1", "models": {"arm": {"asset": {"package_path": "../outside", "package_digest": "0" * 64, "asset_id": "id"}}}}))
    catalog = BrandCatalog(str(tmp_path / "__init__.py"))
    with pytest.raises(ValueError, match="Unsafe package path"):
        catalog.asset("arm", tmp_path)
