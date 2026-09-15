"""关节空间 smoothstep 插值。"""

from __future__ import annotations

import math
from collections.abc import Callable
from threading import Event

import numpy as np


def smoothstep_ratio(step_index: int, step_count: int) -> float:
    """返回 [0, 1] 上的 smoothstep 比例。"""

    if step_count <= 0:
        raise ValueError("step_count 必须为正")
    t = step_index / step_count
    return t * t * (3.0 - 2.0 * t)


def validate_duration(duration: float) -> None:
    """校验 preview 动作时长边界。"""

    if not math.isfinite(duration) or not 0.1 <= duration <= 30.0:
        raise ValueError("duration must be between 0.1 and 30 seconds")


def interpolate_joint_path(
    path: np.ndarray,
    duration: float,
    *,
    stop_event: Event,
    external_stop: Event | None,
    step_s: float = 0.05,
    on_step: Callable[[list[float]], None],
) -> None:
    """沿离散关节路径做 smoothstep 插值并回调每步关节角。"""

    validate_duration(duration)
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] == 0:
        raise ValueError("path 必须是二维关节数组")
    steps = max(1, math.ceil(duration / step_s))
    for i in range(1, steps + 1):
        if stop_event.is_set() or (external_stop is not None and external_stop.is_set()):
            raise RuntimeError("motion interrupted; keep current attachment")
        t = i / steps
        u = t * t * (3.0 - 2.0 * t) * (len(path) - 1)
        n = min(int(u), len(path) - 2)
        q = (path[n] * (1.0 - (u - n)) + path[n + 1] * (u - n)).tolist()
        on_step(q)
