"""Elite CS66 L1 运动学测试。"""

from __future__ import annotations

import numpy as np
from unilab_robot_runtime import ArmMount
from unilab_arm_elite_cs66 import EliteCs66PreviewKinematics, HOME_DEG

MOUNT = ArmMount(
    "elite_left",
    (-0.08078092030064, 0.49262494532318, 0.94498084485531),
)


def test_fk_ik_roundtrip() -> None:
    kin = EliteCs66PreviewKinematics()
    target = (-0.50, 0.55, 1.24)
    q = kin.ik_tcp(MOUNT, target, seed=HOME_DEG, yaw_deg=90.0)
    pose = kin.plate_pose(MOUNT, q)
    np.testing.assert_allclose(pose[:3, 3], target, atol=1e-4)


def test_linear_path_stays_continuous() -> None:
    kin = EliteCs66PreviewKinematics()
    start = kin.ik_tcp(MOUNT, (-0.50, 0.55, 1.24), seed=HOME_DEG, yaw_deg=90.0)
    path = kin.linear_path(MOUNT, start, (-0.50, 0.55, 1.12))
    assert len(path) >= 2
    for left, right in zip(path, path[1:]):
        assert np.max(np.abs(np.asarray(right) - np.asarray(left))) <= 20.0
