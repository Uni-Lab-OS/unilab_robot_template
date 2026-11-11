"""Build all workspace wheels in an external staging directory, without caches in Git."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-root", required=True, type=Path)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--packages", nargs='+', help="Optional exact distributions changed since a previous build")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    work = args.work_root.resolve()
    if work.is_relative_to(repo):
        raise ValueError("Build staging must be outside the source checkout")
    wheels = work / "wheels"
    wheels.mkdir(parents=True, exist_ok=True)
    receipts = []
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    def build(source):
        stage = work / "stage" / source.name
        if stage.exists():
            raise ValueError(f"Use a new external work root: {stage}")
        shutil.copytree(source, stage, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.egg-info", "dist", "build"))
        log = work / (source.name + ".log")
        with log.open("w", encoding="utf-8") as stream:
            result = subprocess.run([sys.executable, "-B", "-X", "utf8", "-c", "import setuptools.build_meta as b; b.build_wheel(" + repr(str(wheels)) + ")"], cwd=stage, env=env, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f"Build failed: {log}")
        return {"package": source.name, "returncode": result.returncode}
    sources = [p for p in sorted((repo / "packages").iterdir()) if (p / "pyproject.toml").is_file()]
    if args.packages:
        unknown=set(args.packages)-{p.name for p in sources}
        if unknown:raise ValueError('Unknown packages: '+str(sorted(unknown)))
        sources=[p for p in sources if p.name in args.packages]
    with ThreadPoolExecutor(max_workers=max(1, min(args.jobs, 8))) as executor:
        receipts = list(executor.map(build, sources))
    document = {"status": "passed", "packages": receipts, "wheels": [{"name": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(wheels.glob("*.whl"))]}
    (work / "receipt.json").write_text(json.dumps(document, indent=2), encoding="utf-8")
    print(json.dumps({"status": "passed", "wheels": len(document["wheels"])}))


if __name__ == "__main__":
    main()
