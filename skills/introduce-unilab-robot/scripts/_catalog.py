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

_PREVIEW_CATALOG: dict[str, dict[str, str]] = {
    "elite-cs66": {
        "distribution": "unilab-arm-elite-cs66",
        "import_root": "unilab_arm_elite_cs66",
        "digest_module": "packages/unilab-arm-elite-cs66/src/unilab_arm_elite_cs66/urdf_providers.py",
        "demo_dir": "docs/demo/catalog-preview-elite-cs66",
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


def list_preview_catalog_slugs() -> list[str]:
    return sorted(_PREVIEW_CATALOG)


@dataclass(frozen=True)
class PreviewCatalogEntry:
    slug: str
    distribution: str
    import_root: str
    package_dir: Path
    demo_dir: Path


def load_preview_catalog_entry(template: Path, slug: str) -> PreviewCatalogEntry:
    spec = _PREVIEW_CATALOG.get(slug)
    if spec is None:
        raise ValueError(
            f"未知 preview catalog slug: {slug}；可选: {', '.join(list_preview_catalog_slugs())}"
        )
    digest_module = template / spec["digest_module"]
    if not digest_module.is_file():
        raise FileNotFoundError(f"找不到 Preview digest 模块: {digest_module}")
    package_dir = digest_module.parents[2]
    demo_dir = template / "skills" / "introduce-unilab-robot" / spec["demo_dir"]
    if not demo_dir.is_dir():
        raise FileNotFoundError(f"找不到 Preview demo 目录: {demo_dir}")
    return PreviewCatalogEntry(
        slug=slug,
        distribution=spec["distribution"],
        import_root=spec["import_root"],
        package_dir=package_dir,
        demo_dir=demo_dir,
    )


def read_preview_source_digest(template: Path, slug: str) -> str:
    import importlib.util

    load_preview_catalog_entry(template, slug)
    digest_module = template / _PREVIEW_CATALOG[slug]["digest_module"]
    spec = importlib.util.spec_from_file_location(
        f"unilab_introduce_{slug.replace('-', '_')}_digest",
        digest_module,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 Preview digest 模块: {digest_module}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    digest = str(getattr(module, "SOURCE_DIGEST", "") or "").strip()
    if len(digest) != 64:
        raise ValueError(f"Preview L1 缺少 SOURCE_DIGEST: {digest_module}")
    return digest


def preview_catalog_dependencies(entry: PreviewCatalogEntry) -> list[str]:
    return [
        f"{entry.distribution}>=0.1,<0.2",
        "unilab-robot-runtime>=0.1,<0.2",
    ]


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
