"""Run with isolated installed Python (-I), denying all source-checkout reads."""
import argparse
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--deny-root", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    args = parser.parse_args()
    os.environ['UNILAB_ASSET_LIBRARY']=str(args.library.resolve())
    denied = os.path.normcase(str(args.deny_root.resolve()))

    def audit(event, values):
        if event == "open" and isinstance(values[0], (str, bytes, os.PathLike)):
            candidate = os.path.normcase(os.path.abspath(os.fsdecode(values[0])))
            if candidate == denied or candidate.startswith(denied + os.sep):
                raise RuntimeError("Source-checkout access forbidden: " + candidate)
    sys.addaudithook(audit)
    rows = []
    for dist in sorted(importlib.metadata.distributions(), key=lambda d: d.metadata["Name"]):
        name = dist.metadata["Name"]
        if not name.startswith("unilab-robot-") or name in ("unilab-robot-contracts", "unilab-robot-runtime"):
            continue
        module = importlib.import_module(name.replace("-", "_"))
        checked_models = 0
        for model in module.models():
            if hasattr(module, "module_definition"):
                definition = module.module_definition(model)
                assert definition["model"] == model
                assert set(definition.get('asset_role', ())) and set(definition['asset_role']) <= {'arm','tool'}, 'Out-of-scope brand module'
                checked_models += 1
            if module.model(model).get("provider") is None:
                try:
                    module.build_moveit_model(model, device_id="forbidden")
                except ValueError:
                    pass
                else:
                    raise AssertionError("Unqualified model accepted")
        for profile in module.source_profiles():
            module.catalog.source_profile(profile["id"])
        rows.append({"brand": name, "models": len(module.models()), "module_definitions_verified": checked_models, "profiles_verified": len(module.source_profiles())})
    providers = []
    for brand, model, legacy in (("dobot", "cr5", "unilab_arm_cr5"), ("dobot", "cr7", "unilab_arm_cr7"), ("rokae", "xmatecr12", "unilab_arm_rokae_cr12")):
        module = importlib.import_module("unilab_robot_" + brand)
        a = module.build_moveit_model(model, device_id="first")
        b = module.build_moveit_model(model, device_id="second")
        prior = importlib.import_module(legacy).build_moveit_model(device_id="first")
        assert a.execution_urdf == prior.execution_urdf and a.srdf == prior.srdf
        assert set(a.qualified_joint_names).isdisjoint(b.qualified_joint_names)
        assert len(a.qualified_joint_names) == 6 and not a.rviz_required
        assert all(Path(p).is_file() for p in a.mesh_paths)
        package = module.resolve_asset(model, args.library)
        providers.append({"brand": brand, "model": model, "source_digest": a.source_digest,
                          "execution_urdf_sha256": hashlib.sha256(a.execution_urdf.encode()).hexdigest(),
                          "legacy_equivalent": True, "distinct_instances": True, "asset": str(package)})
    result = {"status": "passed", "scope": "Installed wheels, frozen profiles, fixed asset packages and headless model construction; no live MoveIt/physics/hardware execution", "source_root_reads_denied": str(args.deny_root), "brands": rows, "providers": providers}
    args.receipt.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": "passed", "brands": len(rows), "models": sum(r['models'] for r in rows), "profiles": sum(r['profiles_verified'] for r in rows), "providers": len(providers)}))


if __name__ == "__main__":
    main()
