"""One-time maintainer import; generated brand wheels never read the source repo.

Run explicitly with --library and optionally --sources. Upstream MoveIt profiles
are frozen, hashed candidates, not launchable providers. Existing tested model
implementations keep their legacy import names within their brand distribution.
"""
from __future__ import annotations

import argparse
import configparser
import hashlib
import json
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROVIDERS = {
    "dobot": {"cr5": "unilab_arm_cr5:build_moveit_model", "cr7": "unilab_arm_cr7:build_moveit_model"},
    "rokae": {"xmatecr12": "unilab_arm_rokae_cr12:build_moveit_model"},
}
BRAND_INIT = '''"""Brand catalog; optional backends load only when explicitly selected."""
from unilab_robot_contracts.brand_catalog import BrandCatalog

catalog = BrandCatalog(__file__)
models = catalog.models
model = catalog.model
build_moveit_model = catalog.build_moveit_model
resolve_asset = catalog.asset
source_profiles = catalog.source_profiles
module_definition = catalog.module_definition
compile_asset_model = catalog.compile_asset_model
fetch_asset = catalog.fetch_asset
hardware_candidates = catalog.hardware_candidates
create_hardware_driver = catalog.create_hardware_driver
'''


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def brand_key(value):
    aliases = {"elite_robots": "elite", "eliterobots": "elite", "yaskawa_motoman": "yaskawa", "doosan_robotics": "doosan", "doosanrobotics": "doosan", "otc_daihen": "otc", "otcdaihen": "otc", "universalrobots": "universal_robots"}
    value = aliases.get(value, value)
    value = {'agilerobotsag':'agilerobots', 'elephantrobotics':'elephant',
             'frankarobotics':'franka', 'fruitcorerobotics':'fruitcore',
             'hanwharobotics':'hanwha', 'hyundairobotics':'hyundai',
             'inovorobotics':'inovo', 'mabiroboticag':'mabirobotic',
             'mitsubishielectric':'mitsubishi', 'mobileindustrialrobotics':'mir',
             'deltaelectronics':'delta'}.get(value,value)
    if not re.fullmatch(r"[a-z][a-z0-9_]*", value):
        raise ValueError(f"Unsupported explicit brand key: {value}")
    return value


def git_commit(path):
    r = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip() if r.returncode == 0 else None


def generate(library, sources, receipt):
    if any((ROOT / 'packages').glob('*/src/*/catalog.json')):
        raise ValueError('Existing brand modules must be preserved. Use --crosswalk for incremental three-source union import.')
    index_bytes = (library / "library.json").read_bytes()
    index = json.loads(index_bytes)
    brands = {}
    for record in index["records"]:
        for target in record.get("targets", []):
            metadata = library / target["package_path"] / "simready_usd/unilab.json"
            if not metadata.is_file():
                continue
            data = json.loads(metadata.read_bytes())
            merge = data.get("merge", {})
            if not merge.get("key"):
                continue
            brand, model = merge["key"]
            brand = brand_key(brand)
            rows = brands.setdefault(brand, {"models": {}, "source_profiles": []})
            binding = {k: target[k] for k in ("asset_id", "package_digest", "package_path")}
            if digest(library / binding["package_path"] / "com.nvidia.simready.packaging.json") != binding["package_digest"]:
                raise ValueError(f"Index/package mismatch: {model}")
            if model in rows["models"] and rows["models"][model]["asset"] != binding:
                raise ValueError(f"Multiple active bindings: {brand}/{model}")
            rows["models"][model] = {
                "title": data["title"], "asset": binding,
                "provider": None, "capabilities": {"planning": "not_qualified", "hardware": "not_qualified"},
                "model_equivalence": "not_verified",
                "_engineering": data,
            }

    # Collect package-local source configuration only. Never copy build/install,
    # meshes, caches or entire Git worktrees; unresolved dependencies stay explicit.
    if sources:
        modules = configparser.ConfigParser()
        modules.read(sources / ".gitmodules", encoding="utf-8")
        source_repos = {}
        for section in modules.sections():
            rel = modules[section].get("path", "")
            if rel.startswith("robots/"):
                source_repos[rel] = {"url": modules[section].get("url"), "commit": None}
        ignored = {".git", "build", "install", "log", "__pycache__", ".venv"}
        allowed = {".yaml", ".yml", ".srdf", ".xacro", ".urdf", ".xml", ".py", ".launch", ".cmake", ".txt", ".md", ".rviz", ".cfg", ".ini"}
        candidates = set()
        for f in (sources / "robots").rglob("package.xml"):
            rel = f.relative_to(sources)
            if any(p in ignored for p in rel.parts):
                continue
            folder = f.parent
            if "moveit" in folder.name.lower() or any(folder.glob("config/*.srdf")) or (folder / "config/kinematics.yaml").is_file():
                candidates.add(folder)
        for folder in sorted(candidates):
            rel = folder.relative_to(sources)
            brand = brand_key(rel.parts[1])
            rows = brands.setdefault(brand, {"models": {}, "source_profiles": []})
            profile_id = re.sub(r"[^a-z0-9_]+", "_", folder.name.lower()).strip("_") + "_" + hashlib.sha256(rel.as_posix().encode()).hexdigest()[:10]
            package_module = "unilab_robot_" + brand
            out = ROOT / "packages" / ("unilab-robot-" + brand.replace("_", "-")) / "src" / package_module / "source_profiles" / profile_id
            copied = []
            for f in sorted(folder.rglob("*")):
                if not f.is_file() or any(p in ignored for p in f.relative_to(folder).parts):
                    continue
                if f.suffix.lower() not in allowed and not f.name.upper().startswith(("LICENSE", "COPYING", "NOTICE")):
                    continue
                local = f.relative_to(folder)
                dst = out / local
                dst.parent.mkdir(parents=True, exist_ok=True)
                dst.write_bytes(f.read_bytes())
                copied.append({"path": local.as_posix(), "sha256": digest(dst), "git_lfs_pointer": dst.read_bytes().startswith(b"version https://git-lfs.github.com/spec/")})
            repo_rel = next((r for r in sorted(source_repos, key=len, reverse=True) if rel.as_posix().startswith(r + "/")), None)
            origin = dict(source_repos.get(repo_rel, {}))
            if repo_rel:
                if not source_repos[repo_rel]["commit"]:
                    source_repos[repo_rel]["commit"] = git_commit(sources / repo_rel)
                origin["commit"] = source_repos[repo_rel]["commit"]
                # Keep repository-level legal notices alongside the selected profile.
                for f in (sources / repo_rel).iterdir():
                    if f.is_file() and f.name.upper().startswith(("LICENSE", "COPYING", "NOTICE")):
                        dst = out / "upstream_notices" / f.name
                        dst.parent.mkdir(parents=True, exist_ok=True)
                        dst.write_bytes(f.read_bytes())
                        copied.append({"path": dst.relative_to(out).as_posix(), "sha256": digest(dst)})
            manifest = {
                "schema": "unilab.moveit-source-profile/1", "id": profile_id,
                "source": {**origin, "path": rel.as_posix(), "authority": "file_hashes; working tree may differ from commit"},
                "status": "source_preserved_not_runtime_enabled", "model_mapping": "unreviewed",
                "dependencies": "package.xml and source URI references retained; execution closure not yet qualified",
                "files": copied,
            }
            write(out / "source.lock.json", manifest)
            rows["source_profiles"].append({"id": profile_id, "manifest": f"source_profiles/{profile_id}/source.lock.json", "sha256": digest(out / "source.lock.json"), "status": manifest["status"]})

    for brand, providers in PROVIDERS.items():
        rows = brands.setdefault(brand, {"models": {}, "source_profiles": []})
        for model, provider in providers.items():
            row = rows["models"].setdefault(model, {"title": model, "asset": None, "model_equivalence": "not_verified"})
            row.update(provider=provider, capabilities={"planning": "existing_provider_contract", "hardware": "requires_explicit_backend_qualification"})

    # Explicit reviewed identity candidate: never select the unrelated Dobot
    # CR12 or a differently suffixed ROKAE product by fuzzy model name.
    cr12_id = "f09d1a8c-7324-5c5e-875b-e53b363000a6"
    for record in index["records"]:
        for target in record.get("targets", []):
            if target["asset_id"] == cr12_id:
                brands["rokae"]["models"]["xmatecr12"]["asset"] = {k: target[k] for k in ("asset_id", "package_digest", "package_path")}
                brands["rokae"]["models"]["xmatecr12"]["_engineering"] = json.loads((library / target["package_path"] / "simready_usd/unilab.json").read_bytes())

    for brand, rows in sorted(brands.items()):
        dist = "unilab-robot-" + brand.replace("_", "-")
        module = "unilab_robot_" + brand
        package = ROOT / "packages" / dist
        module_root = package / "src" / module
        module_root.mkdir(parents=True, exist_ok=True)
        (module_root / "__init__.py").write_text(BRAND_INIT, encoding="utf-8")
        for model, row in rows["models"].items():
            # Model key is semantic; filesystem name is bounded and collision-safe.
            slug = re.sub(r"[^a-z0-9_-]+", "_", model.lower()).strip("_")
            if slug != model:
                slug += "_" + hashlib.sha256(model.encode()).hexdigest()[:8]
            folder = module_root / "models" / slug
            engineering = row.pop("_engineering", {})
            source_types = sorted({str(s.get("type", "unknown")) for s in engineering.get("parameters", {}).get("source_models", [])})
            candidates = []
            normalized_model = re.sub(r"[^a-z0-9]", "", model)
            for profile in rows["source_profiles"]:
                lock = json.loads((module_root / profile["manifest"]).read_bytes())
                basename = Path(lock["source"]["path"]).name.lower()
                for ending in ("_moveit_config", "_moveit", "_config"):
                    if basename.endswith(ending):
                        basename = basename[:-len(ending)]
                        break
                if re.sub(r"[^a-z0-9]", "", basename) == normalized_model:
                    candidates.append({"id": profile["id"], "status": "exact_name_candidate_not_qualified"})
            representation = {"selected_usd_vs_provider": "not_verified"}
            if row.get("provider") and row.get("asset"):
                python_module = row["provider"].split(":")[0]
                local_urdf = next((package / "src" / python_module / "models").glob("*_robot.urdf"), None)
                if local_urdf:
                    def movable(path):
                        return [(v.attrib, [(c.tag, c.attrib) for c in v]) for v in ET.parse(path).getroot().findall("joint") if v.get("type") != "fixed"]
                    nominal = movable(local_urdf)
                    matches = []
                    for source in engineering.get("sources", []):
                        if str(source.get("path", "")).endswith(".urdf"):
                            p = library / row["asset"]["package_path"] / source["path"]
                            if nominal == movable(p):
                                matches.append({"path": source["path"], "sha256": digest(p)})
                    representation.update(provider_urdf_sha256=digest(local_urdf), matching_source_joint_definitions=matches,
                                          scope="Serialized movable-joint declarations only; not selected USD geometry/frame or hardware equivalence")
            definition = {
                "schema": "unilab.robot-module/1", "brand": brand, "model": model, "title": row["title"],
                "ownership": "brand package maintains module; standard library publishes fixed mechanical representations",
                "asset": row.get("asset"), "provider": row.get("provider"), "capabilities": row["capabilities"],
                "composition": {"root_asset": row.get("asset"), "scope": "Selected reusable assembly only; site installation and station coordination belong to domain package"},
                "motion_model": {"source_types": source_types, "parameters": engineering.get("parameters", {}), "configuration": engineering.get("configuration"), "entrypoints": engineering.get("entrypoints", {})},
                "behavior": engineering.get("behavior_binding", {}),
                "drivers": {"implementation_module": row["provider"].split(":")[0] if row.get("provider") else None,
                            "status": "existing_explicit_adapters" if brand == "dobot" and row.get("provider") else "not_implemented"},
                "planning": {"provider": row.get("provider"), "source_profile_candidates": candidates,
                             "scope": "Source profile is never selected automatically or treated as an executable provider"},
                "representation_correspondence": representation,
                "source_provenance": {"sources": engineering.get("sources", []), "merge": engineering.get("merge", {})},
            }
            write(folder / "module.json", definition)
            write(folder / "assets.lock.json", {"schema": "unilab.robot-module-assets/1", "asset": row.get("asset")})
            row["module_path"] = (folder / "module.json").relative_to(module_root).as_posix()
            row["module_sha256"] = digest(folder / "module.json")
        write(module_root / "catalog.json", {"schema": "unilab.robot-brand/1", "brand": brand, "library_index_sha256": hashlib.sha256(index_bytes).hexdigest(), **rows})
        legacy = {"dobot": ["unilab_arm_cr5", "unilab_arm_cr7"], "rokae": ["unilab_arm_rokae_cr12"]}.get(brand, [])
        data_rules = f'{module} = ["catalog.json", "models/**/*", "source_profiles/**/*"]\n' + "".join(f'{m} = ["models/**/*"]\n' for m in legacy)
        (package / "pyproject.toml").write_text(f'''[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "{dist}"
version = "0.1.0"
description = "Uni-Lab {brand} brand models and reviewed capability bindings"
requires-python = ">=3.11"
dependencies = ["PyYAML>=6.0", "unilab-robot-contracts>=0.1,<0.2"]

[tool.setuptools.packages.find]
where = ["src"]
include = ["{module}"{''.join(', "'+m+'*"' for m in legacy)}]

[tool.setuptools.package-data]
{data_rules}''', encoding="utf-8")
    summary = {"schema": "unilab.brand-import/1", "brands": len(brands), "models": sum(len(v["models"]) for v in brands.values()), "source_profiles": sum(len(v["source_profiles"]) for v in brands.values()), "library_index_sha256": hashlib.sha256(index_bytes).hexdigest(), "runtime_upstream_checkout_required": False,
               "packages": ["unilab-robot-" + b.replace("_", "-") for b in sorted(brands)]}
    write(receipt, summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--sources", type=Path, help="Optional one-time import input; never used by generated wheels")
    parser.add_argument("--receipt", required=True, type=Path)
    parser.add_argument("--crosswalk", type=Path, help="Three-source union crosswalk; incrementally preserves existing modules")
    parser.add_argument("--scope-report", type=Path, help="Required arms/tools classification report for union maintenance")
    args = parser.parse_args()
    if args.crosswalk:
        from extend_brand_union import extend
        extend(args.library.resolve(), args.crosswalk.resolve(), args.receipt,args.scope_report)
    else:
        generate(args.library.resolve(), args.sources.resolve() if args.sources else None, args.receipt)
