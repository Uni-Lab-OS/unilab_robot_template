"""Validated mechanical ownership and execution binding, without a task scheduler.

The OS graph owns placement. This declaration identifies its mechanical parts,
existing tool geometry and control groups; it does not instantiate another tree.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Callable, Mapping
import xml.etree.ElementTree as ET


def digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


class Composition:
    """One arm, optional rail, and the installed tool, with exclusive ownership."""

    def __init__(self, document: Mapping):
        data = deepcopy(dict(document))
        if data.get("schema") != "unilab.workcell-composition/1":
            raise ValueError("Unsupported workcell composition schema")
        if not isinstance(data.get("identity"), str) or not data["identity"]:
            raise ValueError("Composition identity is required")
        parts = data.get("parts", {})
        if set(parts) not in ({"arm", "tool"}, {"rail", "arm", "tool"}):
            raise ValueError("Composition requires arm/tool and an optional rail")
        joints, geometry = {}, {}
        for role, part in parts.items():
            if not part.get("model"):
                raise ValueError(f"Missing model identity: {role}")
            for joint in part.get("joints", []):
                if not isinstance(joint, str) or not joint or joint in joints:
                    raise ValueError(f"Duplicate or invalid joint ownership: {joint}")
                joints[joint] = role
            for selector in part.get("geometry", []):
                if not isinstance(selector, str) or not selector or selector in geometry:
                    raise ValueError(f"Duplicate or invalid geometry ownership: {selector}")
                geometry[selector] = role
        tool = parts["tool"]
        if tool.get("binding") not in ("embedded", "asset"):
            raise ValueError("Tool must explicitly bind embedded geometry or an asset")
        if not tool.get("geometry") or not tool.get("mount_frame") or not tool.get("tcp_source"):
            raise ValueError("Tool geometry, mount frame and TCP source are required")
        if tool["binding"] == "embedded" and tool.get("instantiate_asset"):
            raise ValueError("Embedded tool cannot instantiate another asset")
        if tool["binding"] == "asset" and not tool.get("asset_lock"):
            raise ValueError("Independent tool requires a pinned asset lock")
        if not parts["arm"].get("joints"):
            raise ValueError("Arm joint ownership is required")
        modes = data.get("execution", {})
        if not modes:
            raise ValueError("Explicit execution bindings are required")
        for mode, binding in modes.items():
            ExecutionBinding(mode, binding, frozenset(parts))
        self._document = data
        self.digest = digest(data)

    @classmethod
    def read(cls, path: str | Path) -> Composition:
        return cls(json.loads(Path(path).read_bytes()))

    @property
    def document(self) -> dict:
        return deepcopy(self._document)

    def execution(self, mode: str) -> ExecutionBinding:
        if mode not in self._document["execution"]:
            raise ValueError(f"Execution mode not configured: {mode}")
        return ExecutionBinding(mode, self._document["execution"][mode],
                                frozenset(self._document["parts"]))

    def validate_model(self, urdf: str, *, external_geometry: Mapping[str, str] | None = None) -> dict:
        """Check compiled URDF ownership and optional CAD bodies bound by the caller.

        External geometry maps a declared selector to its actual unique body ID.
        This validates ownership, not geometric equivalence or collision freedom.
        """
        tree = ET.fromstring(urdf)
        links = [e.get("name") for e in tree.findall("link")]
        joints = {e.get("name"): e for e in tree.findall("joint")}
        if len(links) != len(set(links)) or len(joints) != len(tree.findall("joint")):
            raise ValueError("Duplicate compiled link or joint")
        declared = {}
        for role, part in self._document["parts"].items():
            for name in part.get("joints", []):
                if name not in joints or joints[name].get("type") == "fixed":
                    raise ValueError(f"Missing movable {role} joint: {name}")
                if role == "rail" and joints[name].get("type") != "prismatic":
                    raise ValueError("Rail joint must be prismatic")
                declared[name] = role
        movable = {n for n, j in joints.items() if j.get("type") != "fixed"}
        if movable != set(declared):
            raise ValueError(f"Unowned compiled joints: {sorted(movable - set(declared))}")
        tool = self._document["parts"]["tool"]
        if tool["mount_frame"] not in links:
            raise ValueError("Tool mount frame missing from compiled model")
        resolved = {}
        for selector in tool["geometry"]:
            if selector.startswith("link:"):
                name = selector[5:]
                node = tree.find(f"link[@name='{name}']")
                if node is None or (node.find("visual") is None and node.find("collision") is None):
                    raise ValueError(f"Tool geometry missing: {selector}")
                resolved[selector] = selector
            elif external_geometry is not None:
                if selector not in external_geometry:
                    raise ValueError(f"External tool geometry missing: {selector}")
                resolved[selector] = external_geometry[selector]
        if len(set(resolved.values())) != len(resolved):
            raise ValueError("Multiple tool selectors reference the same body")
        return {"composition_digest": self.digest, "model_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
                "joint_owners": declared, "tool_geometry": resolved,
                "external_geometry_pending": [s for s in tool["geometry"] if s not in resolved]}


class ExecutionBinding:
    """Capability gate over existing executors; never journals, retries or settles inventory."""

    _MODES = {"simulation_trajectory", "independent_drives", "plc_program"}
    _CAPABILITIES = {"program", "trajectory", "point", "tool", "observe", "stop"}

    def __init__(self, mode: str, binding: Mapping, parts: frozenset[str]):
        if mode not in self._MODES:
            raise ValueError(f"Unsupported execution mode: {mode}")
        capabilities = frozenset(binding.get("capabilities", []))
        if not capabilities or capabilities - self._CAPABILITIES:
            raise ValueError("Invalid execution capabilities")
        groups = binding.get("control_groups", {})
        owned = [role for roles in groups.values() for role in roles]
        if len(owned) != len(set(owned)) or set(owned) != set(parts) or any(not k for k in groups):
            raise ValueError("Each mechanical part must have exactly one control owner")
        if mode == "plc_program" and (len(groups) != 1 or capabilities - {"program", "observe", "stop"}):
            raise ValueError("PLC program owns the whole mechanism; independent motion is not supported")
        if mode == "simulation_trajectory" and "trajectory" not in capabilities:
            raise ValueError("Trajectory simulation requires a trajectory executor")
        self.mode = mode
        self.capabilities = capabilities
        self.control_groups = deepcopy(groups)

    def require(self, capability: str) -> None:
        if capability not in self.capabilities:
            raise ValueError(f"{self.mode} does not support {capability}")

    def dispatch(self, capability: str, executor: Callable, /, *args, **kwargs):
        """Call the selected existing backend exactly once, preserving its errors/results."""
        self.require(capability)
        return executor(*args, **kwargs)
