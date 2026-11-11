"""Brand/model discovery and exact asset binding, without upstream checkout access.

Source profiles are inert import candidates. Only explicit provider entries may
build executable model bundles; a matching name never grants that capability.
"""
from __future__ import annotations

import importlib
import hashlib
import json
from pathlib import Path
from typing import Any


class BrandCatalog:
    def __init__(self, package_file: str) -> None:
        self.root = Path(package_file).resolve().parent

    def _read(self) -> dict[str, Any]:
        data = json.loads((self.root / "catalog.json").read_text(encoding="utf-8"))
        if data.get("schema") != "unilab.robot-brand/1":
            raise ValueError("Unsupported brand catalog")
        return data

    def models(self) -> tuple[str, ...]:
        return tuple(sorted(self._read()["models"]))

    def model(self, name: str) -> dict[str, Any]:
        try:
            return self._read()["models"][name]
        except KeyError as exc:
            raise ValueError(f"Unknown exact model: {name}") from exc

    def provider(self, name: str) -> Any:
        row = self.model(name)
        if row.get("module_path"):
            self.module_definition(name)
        entry = row.get("provider")
        if not entry:
            raise ValueError(f"Model has no validated provider: {name}")
        module, function = entry.split(":", 1)
        return getattr(importlib.import_module(module), function)

    def module_definition(self, name: str) -> dict[str, Any]:
        """Return the brand-owned complete module definition after digest checking."""
        from ._asset_package import inside

        row = self.model(name)
        path = inside(self.root, row["module_path"])
        value = path.read_bytes()
        if hashlib.sha256(value).hexdigest() != row["module_sha256"]:
            raise ValueError("Module definition changed")
        data = json.loads(value)
        if data.get("model") != name or data.get("asset") != row.get("asset") or data.get("provider") != row.get("provider"):
            raise ValueError("Module definition/catalog mismatch")
        return data

    def build_moveit_model(self, name: str, **kwargs: Any) -> Any:
        # Preserve each provider's established digest/frame/unit checks.
        return self.provider(name)(**kwargs)

    def compile_asset_model(self, name: str, library_root: str | Path, output: str | Path, **kwargs: Any) -> Any:
        """Compile locked USD into external MoveIt cache; never reuse vendor-name guesses."""
        from unilab_robot_runtime.asset_moveit import compile_asset_model
        definition = self.module_definition(name)
        if definition.get('asset_role') == ['tool']:
            raise ValueError('End-effectors use fixed asset binding and attachment; no standalone arm planner')
        return compile_asset_model(self, name, library_root, output, **kwargs)

    def fetch_asset(self, name: str, cache_root: str | Path, read_object: Any) -> Path:
        from .asset_cache import fetch_asset
        return fetch_asset(self.module_definition(name)['asset'], cache_root, read_object)

    def hardware_candidates(self, name: str) -> tuple[dict[str, Any], ...]:
        """Evidence-backed driver candidates; never a claim of hardware support."""
        return tuple(self.module_definition(name).get('drivers', {}).get('hardware_candidates', ()))

    def create_hardware_driver(self, name: str, driver_id: str, **kwargs: Any) -> Any:
        """Build an explicitly selected adapter without connecting or moving."""
        from unilab_robot_runtime.hardware_driver import create_driver
        return create_driver(self, name, driver_id, **kwargs)

    def asset(self, name: str, library_root: str | Path) -> Path:
        """Resolve a fixed package in an explicitly supplied local/cache root.

        No downloads, latest lookup, upstream Git access or implicit substitutions.
        This establishes package integrity, not provider/asset model equivalence.
        """
        from ._asset_package import inside, verify

        binding = self.model(name).get("asset")
        if not binding:
            raise ValueError(f"No published asset bound to {name}")
        root = Path(library_root).resolve()
        package = inside(root, binding["package_path"])
        receipt = verify(package, binding["package_digest"])
        if receipt["asset_id"] != binding["asset_id"]:
            raise ValueError("Asset identity differs from model binding")
        return package

    def source_profiles(self) -> tuple[dict[str, Any], ...]:
        """Inventory of frozen upstream configuration candidates, never executed."""
        return tuple(self._read().get("source_profiles", ()))

    def source_profile(self, profile_id: str) -> dict[str, Any]:
        """Read a frozen candidate after checking every preserved file hash."""
        from ._asset_package import inside

        matches = [p for p in self.source_profiles() if p["id"] == profile_id]
        if len(matches) != 1:
            raise ValueError("Unknown or ambiguous source profile")
        row = matches[0]
        path = inside(self.root, row["manifest"])
        if hashlib.sha256(path.read_bytes()).hexdigest() != row["sha256"]:
            raise ValueError("Source profile manifest changed")
        data = json.loads(path.read_bytes())
        for file in data["files"]:
            value = inside(path.parent, file["path"]).read_bytes()
            if hashlib.sha256(value).hexdigest() != file["sha256"]:
                raise ValueError("Source profile file changed: " + file["path"])
        return data
