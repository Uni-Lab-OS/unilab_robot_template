"""unilab_robot_template L1 机械臂 catalog 索引。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class ArmCatalogEntry:
    slug: str
    distribution: str
    import_root: str
    provider: str
    model_yaml: Path
    package_dir: Path


_CATALOG: dict[str, dict[str, str]] = {
    "cr5": {
        "distribution": "unilab-arm-cr5",
        "import_root": "unilab_arm_cr5",
        "model_yaml": "packages/unilab-arm-cr5/src/unilab_arm_cr5/models/model.yaml",
    },
    "cr7": {
        "distribution": "unilab-arm-cr7",
        "import_root": "unilab_arm_cr7",
        "model_yaml": "packages/unilab-arm-cr7/src/unilab_arm_cr7/models/model.yaml",
    },
}


def template_root(start: Path) -> Path | None:
    """向上查找 unilab_robot_template 根目录。"""

    for parent in [start, *start.parents]:
        if (parent / "packages" / "unilab-arm-cr5").is_dir() and (
            parent / "skills" / "introduce-unilab-robot"
        ).is_dir():
            return parent
        if parent.name == "unilab_robot_template" and (parent / "packages").is_dir():
            return parent
    return None


def list_catalog_slugs() -> list[str]:
    return sorted(_CATALOG)


def load_catalog_entry(template: Path, slug: str) -> ArmCatalogEntry:
    spec = _CATALOG.get(slug)
    if spec is None:
        raise ValueError(f"未知 catalog slug: {slug}；可选: {', '.join(list_catalog_slugs())}")
    model_yaml = template / spec["model_yaml"]
    if not model_yaml.is_file():
        raise FileNotFoundError(f"找不到 model.yaml: {model_yaml}")
    import_root = spec["import_root"]
    return ArmCatalogEntry(
        slug=slug,
        distribution=spec["distribution"],
        import_root=import_root,
        provider=f"{import_root}:build_moveit_model",
        model_yaml=model_yaml,
        package_dir=model_yaml.parents[2],
    )


def read_source_digest(model_yaml: Path) -> str:
    payload = yaml.safe_load(model_yaml.read_text(encoding="utf-8"))
    digest = str((payload or {}).get("source", {}).get("sha256") or "").strip()
    if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
        raise ValueError(f"model.yaml 缺少合法 source.sha256: {model_yaml}")
    return digest


def catalog_dependencies(entry: ArmCatalogEntry) -> list[str]:
    return [
        f"{entry.distribution}>=0.1,<0.2",
        "unilab-robot-runtime>=0.1,<0.2",
    ]
