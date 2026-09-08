"""Digest helpers for locked robot model assets."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


def sha256_bytes(data: bytes) -> str:
    """Return lowercase SHA-256 hex digest."""

    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    """Hash raw file bytes."""

    return sha256_bytes(path.read_bytes())


def sha256_normalized_yaml(path: Path) -> str:
    """Hash YAML descriptor with CRLF normalized to LF."""

    normalized = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return sha256_bytes(normalized)


def compute_topology_digest(*, payload: dict[str, object]) -> str:
    """Build stable kinematic topology digest independent of mount pose."""

    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    return sha256_bytes(encoded.encode("utf-8"))
