"""graph JSON scaffold：导轨 parent + 机械臂 child。"""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any


def _rail_node(*, domain_pkg: str, rail_id: str, arm_id: str) -> dict[str, Any]:
    return {
        "id": rail_id,
        "name": f"{rail_id} rail",
        "children": [arm_id],
        "parent": None,
        "type": "device",
        "class": f"community.{domain_pkg}.{rail_id}",
        "position": {"x": 0, "y": 0, "z": 0},
        "config": {
            "standard_execution_backend": "mock",
            "rendering": {"kind": "rail", "dimensionsMm": [2300, 180, 150]},
        },
        "data": {},
    }


def _arm_node(*, domain_pkg: str, rail_id: str, arm_id: str) -> dict[str, Any]:
    return {
        "id": arm_id,
        "name": f"{arm_id} arm",
        "children": [],
        "parent": rail_id,
        "type": "device",
        "class": f"community.{domain_pkg}.{arm_id}",
        "position": {
            "position": {"x": 0, "y": 0, "z": 200},
            "rotation": {"unit": "deg", "x": 0, "y": 0, "z": 180},
        },
        "config": {
            "standard_execution_backend": "moveit_sim",
            "rendering": {"kind": "robot", "dimensionsMm": [900, 900, 1200]},
        },
        "data": {},
    }


def build_graph_document(
    *,
    domain_pkg: str,
    rail_id: str,
    arm_id: str,
    existing: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """生成或合并仅含 rail+arm 的 graph 文档。"""

    rail = _rail_node(domain_pkg=domain_pkg, rail_id=rail_id, arm_id=arm_id)
    arm = _arm_node(domain_pkg=domain_pkg, rail_id=rail_id, arm_id=arm_id)

    if existing is None:
        return {"nodes": [rail, arm], "links": []}

    merged = deepcopy(existing)
    nodes = merged.setdefault("nodes", [])
    if not isinstance(nodes, list):
        raise ValueError("graph.nodes 必须是数组")

    by_id = {
        str(node.get("id") or ""): node
        for node in nodes
        if isinstance(node, dict) and str(node.get("id") or "")
    }
    for fresh in (rail, arm):
        node_id = fresh["id"]
        if node_id in by_id:
            current = by_id[node_id]
            current.update(fresh)
            if node_id == rail_id:
                children = set(current.get("children") or [])
                children.add(arm_id)
                current["children"] = sorted(children)
        else:
            nodes.append(fresh)
            by_id[node_id] = fresh
    merged.setdefault("links", [])
    return merged


def scaffold_graph(
    domain: Path,
    *,
    domain_pkg: str,
    rail_id: str,
    arm_id: str,
    graph_rel: str,
    apply: bool,
) -> dict[str, Any]:
    domain = domain.resolve()
    graph_path = domain / Path(graph_rel)
    existing: dict[str, Any] | None = None
    if graph_path.is_file():
        existing = json.loads(graph_path.read_text(encoding="utf-8"))
    document = build_graph_document(
        domain_pkg=domain_pkg,
        rail_id=rail_id,
        arm_id=arm_id,
        existing=existing,
    )
    rel = str(graph_path.relative_to(domain)).replace("\\", "/")
    changed = existing != document if existing is not None else True
    if apply:
        graph_path.parent.mkdir(parents=True, exist_ok=True)
        graph_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return {
        "graph_path": rel,
        "changed": changed,
        "node_ids": [rail_id, arm_id],
    }
