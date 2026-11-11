"""Validate OS local-mount snapshots without importing OS or a domain package."""
import hashlib
import json
import numpy as np


def local_mounts(state, urdf, expected_bindings, link_poses):
    snapshot = state.get("attachment_state")
    if snapshot is None:
        return None  # Legacy artifacts retain explicit world-to-link reconstruction.
    value = {key: item for key, item in snapshot.items() if key != "revision"}
    digest = hashlib.sha256(json.dumps(value, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if (value.get("schema") != "unilab.attachment-state/v1"
            or digest != snapshot.get("revision")
            or value.get("robot_urdf_sha256") != hashlib.sha256(urdf.encode()).hexdigest()):
        raise ValueError("Attachment snapshot identity differs")
    mounts = value["bindings"]
    expected = {b["body"]: b["link_name"] for b in expected_bindings}
    if set(mounts) != set(expected):
        raise ValueError("Attachment snapshot custody/inventory differs")
    result = {}
    for body, link in expected.items():
        binding = mounts[body]
        if binding["link_name"] != link:
            raise ValueError("Attachment snapshot link differs")
        matrix = np.asarray(binding["link_from_body"], dtype=float)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1], rtol=0, atol=1e-12)
                or not np.allclose(matrix[:3,:3].T @ matrix[:3,:3], np.eye(3), rtol=0, atol=1e-8)
                or abs(np.linalg.det(matrix[:3,:3])-1) > 1e-8):
            raise ValueError("Attachment snapshot transform is not rigid")
        if body not in state["body_poses"] or link not in link_poses:
            raise ValueError("Attachment snapshot missing current body/link pose")
        if not np.allclose(np.asarray(link_poses[link]) @ matrix,
                           state["body_poses"][body], rtol=0, atol=1e-8):
            raise ValueError("Attachment snapshot projection differs")
        result[body] = matrix
    return result
