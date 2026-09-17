"""Preview 执行器 TCP 点动（无 MoveIt commissioning 端口）。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from .preview_arm_device import PreviewArmDevice


def jog_tcp_once_preview(
    device: PreviewArmDevice,
    *,
    axis: str,
    direction: str,
    step: float,
    frame_ref: str,
    duration: float = 1.0,
) -> dict[str, object]:
    """在 Preview 臂上执行一次有限 TCP 点动。"""

    normalized_axis = str(axis).strip().lower()
    normalized_direction = str(direction).strip().lower()
    normalized_frame = str(frame_ref).strip()
    normalized_step = float(step)
    if normalized_frame not in {"arm_base", "tool"}:
        raise ValueError("frame_ref 只允许 arm_base 或 tool")
    if not math.isfinite(normalized_step) or normalized_step <= 0.0:
        raise ValueError("TCP Jog 单步必须为正的有限数")
    if normalized_direction not in {"positive", "negative"}:
        raise ValueError("direction 只允许 positive 或 negative")
    signed = normalized_step if normalized_direction == "positive" else -normalized_step

    mount = device._mount
    kinematics = device._kinematics
    rail_y = float(getattr(device, "rail", 0.0) or 0.0)
    seed = device.joints_deg
    current = np.asarray(kinematics.fk_tcp(mount, seed, rail_y), dtype=float)
    target = _apply_tcp_delta(
        current,
        axis=normalized_axis,
        signed_step=signed,
        frame_ref=normalized_frame,
    )
    solved = _ik_tcp_transform(kinematics, mount, target, seed=seed, rail_y=rail_y)
    result = device.moveJ(*solved, duration=duration)
    return {
        "success": result.get("status") == "completed",
        "status": result.get("status"),
        "joints_deg": result.get("joints_deg", []),
        "mode": result.get("mode"),
    }


def _apply_tcp_delta(
    transform: np.ndarray,
    *,
    axis: str,
    signed_step: float,
    frame_ref: str,
) -> np.ndarray:
    rotation = transform[:3, :3].copy()
    translation = transform[:3, 3].copy()
    if axis in {"x", "y", "z"}:
        delta = np.zeros(3, dtype=float)
        delta[{"x": 0, "y": 1, "z": 2}[axis]] = signed_step / 1000.0
        if frame_ref == "tool":
            translation = translation + rotation @ delta
        else:
            translation = translation + delta
    elif axis in {"rx", "ry", "rz"}:
        euler_axis = axis[-1]
        delta_rotation = Rotation.from_euler(
            euler_axis,
            math.radians(signed_step),
        ).as_matrix()
        if frame_ref == "tool":
            rotation = rotation @ delta_rotation
        else:
            rotation = delta_rotation @ rotation
    else:
        raise ValueError(f"未知 TCP 轴：{axis}")
    target = np.eye(4, dtype=float)
    target[:3, :3] = rotation
    target[:3, 3] = translation
    return target


def _ik_tcp_transform(
    kinematics: object,
    mount: object,
    target: np.ndarray,
    *,
    seed: list[float],
    rail_y: float,
) -> list[float]:
    fk = getattr(kinematics, "fk_tcp")

    def residual(q: np.ndarray) -> np.ndarray:
        pose = np.asarray(fk(mount, np.rad2deg(q), rail_y), dtype=float)
        return np.r_[
            pose[:3, 3] - target[:3, 3],
            0.3 * Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec(),
        ]

    starts = [
        seed,
        list(getattr(kinematics, "home_deg", seed)),
        (0.0, -90.0, 90.0, -90.0, 90.0, 0.0),
        (180.0, -90.0, -90.0, -90.0, 90.0, 0.0),
        (90.0, -180.0, 90.0, -180.0, 90.0, 0.0),
        (-90.0, -90.0, 90.0, -90.0, 90.0, 0.0),
    ]
    errors: list[str] = []
    for initial in starts:
        try:
            result = least_squares(
                residual,
                np.deg2rad(initial),
                max_nfev=200,
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
            )
            q = np.asarray(seed) + (np.rad2deg(result.x) - seed + 180) % 360 - 180
            if np.linalg.norm(residual(np.deg2rad(q))) <= 1e-4 and np.max(np.abs(q)) <= 360:
                return q.tolist()
        except ValueError as exc:
            errors.append(str(exc))
    detail = "; ".join(errors) if errors else "IK 未收敛"
    raise ValueError(f"Preview TCP Jog 无法到达目标：{detail}")
