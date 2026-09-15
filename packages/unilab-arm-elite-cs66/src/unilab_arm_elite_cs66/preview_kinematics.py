"""Elite CS66 解析 FK/IK，与可视 URDF 一致。"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from unilab_robot_runtime.preview.types import ArmMount

from .urdf_providers import HOME_DEG, ORIGINS

GRASP = np.eye(4)
GRASP[:3, :3] = np.diag([1.0, -1.0, -1.0])
GRASP[2, 3] = 0.12

_FIXED: list[np.ndarray] = []
for origin in ORIGINS:
    transform = np.eye(4)
    transform[:3, 3] = origin[:3]
    transform[:3, :3] = Rotation.from_euler("xyz", origin[3:]).as_matrix()
    _FIXED.append(transform)


class EliteCs66PreviewKinematics:
    """L1 PreviewKinematics 实现。"""

    home_deg = HOME_DEG

    def fk_tcp(
        self,
        mount: ArmMount,
        q_deg,
        rail_y: float = 0.0,
    ) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, 3] = mount.base_xyz
        transform[1, 3] += rail_y
        for origin, angle in zip(_FIXED, np.deg2rad(q_deg)):
            rz = np.eye(4)
            c, s = np.cos(angle), np.sin(angle)
            rz[:2, :2] = ((c, -s), (s, c))
            transform = transform @ origin @ rz
        return transform

    def plate_pose(self, mount: ArmMount, q_deg, rail_y: float = 0.0) -> np.ndarray:
        return self.fk_tcp(mount, q_deg, rail_y) @ GRASP

    def ik_tcp(
        self,
        mount: ArmMount,
        xyz,
        *,
        seed=None,
        yaw_deg: float = 0.0,
        rail_y: float = 0.0,
    ) -> list[float]:
        seed = list(seed or HOME_DEG)
        target = np.eye(4)
        target[:3, 3] = xyz
        target[:3, :3] = Rotation.from_euler(
            "z", yaw_deg, degrees=True
        ).as_matrix()

        def residual(q):
            pose = self.plate_pose(mount, np.rad2deg(q), rail_y)
            return np.r_[
                pose[:3, 3] - target[:3, 3],
                0.3
                * Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec(),
            ]

        starts = [
            seed,
            (0.0, -90.0, 90.0, -90.0, 90.0, 0.0),
            (180.0, -90.0, -90.0, -90.0, 90.0, 0.0),
            (90.0, -180.0, 90.0, -180.0, 90.0, 0.0),
            (-90.0, -90.0, 90.0, -90.0, 90.0, 0.0),
        ]
        for initial in starts:
            result = least_squares(
                residual,
                np.deg2rad(initial),
                max_nfev=150,
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
            )
            q = np.asarray(seed) + (np.rad2deg(result.x) - seed + 180) % 360 - 180
            if np.linalg.norm(residual(np.deg2rad(q))) <= 1e-5 and np.max(np.abs(q)) <= 360:
                return q.tolist()
        raise ValueError(f"{mount.arm_id} 无法到达板中心 {xyz}")

    def linear_path(
        self,
        mount: ArmMount,
        start_q,
        target_xyz,
        spacing: float = 0.012,
        rail_y: float = 0.0,
    ) -> list[list[float]]:
        start = self.plate_pose(mount, start_q, rail_y)[:3, 3]
        yaw = float(
            Rotation.from_matrix(self.plate_pose(mount, start_q, rail_y)[:3, :3])
            .as_euler("xyz", degrees=True)[2]
        )
        target = np.asarray(target_xyz)
        count = max(2, int(np.ceil(np.linalg.norm(target - start) / spacing)))
        path = [list(start_q)]
        for ratio in np.linspace(0.0, 1.0, count + 1)[1:]:
            q = self.ik_tcp(
                mount,
                start + (target - start) * ratio,
                seed=path[-1],
                yaw_deg=yaw,
                rail_y=rail_y,
            )
            if np.max(np.abs(np.asarray(q) - path[-1])) > 20:
                raise ValueError("笛卡尔路径发生关节分支跳变")
            path.append(q)
        return path

    def carry_path(
        self,
        mount: ArmMount,
        start_q,
        target_xyz,
        rail_y: float = 0.0,
    ) -> list[list[float]]:
        base = np.asarray(mount.base_xyz) + np.array([0.0, rail_y, 0.0])
        start = self.plate_pose(mount, start_q, rail_y)[:3, 3]
        yaw = float(
            Rotation.from_matrix(self.plate_pose(mount, start_q, rail_y)[:3, :3])
            .as_euler("xyz", degrees=True)[2]
        )
        target = np.asarray(target_xyz)
        a, b = start[:2] - base[:2], target[:2] - base[:2]
        theta = np.arctan2(a[1], a[0])
        delta = (np.arctan2(b[1], b[0]) - theta + np.pi) % (2 * np.pi) - np.pi
        path = [list(start_q)]
        for t in np.linspace(0, 1, 100)[1:]:
            radius = np.linalg.norm(a) * (1 - t) + np.linalg.norm(b) * t
            point = [
                base[0] + radius * np.cos(theta + delta * t),
                base[1] + radius * np.sin(theta + delta * t),
                start[2] * (1 - t) + target[2] * t,
            ]
            q = self.ik_tcp(
                mount,
                point,
                seed=path[-1],
                yaw_deg=yaw,
                rail_y=rail_y,
            )
            if np.max(np.abs(np.asarray(q) - path[-1])) > 20:
                raise ValueError("搬运路径发生关节分支跳变")
            path.append(q)
        return path


@lru_cache(maxsize=1)
def default_kinematics() -> EliteCs66PreviewKinematics:
    return EliteCs66PreviewKinematics()
