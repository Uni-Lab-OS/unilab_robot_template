"""ExecutorObservation 适配器单测。"""

from __future__ import annotations

import math
import threading

from unilab_robot_runtime.device_card.observation_adapters import observation_from_preview_arm
from unilab_robot_runtime.preview.preview_arm_device import PreviewArmDevice
from unilab_robot_runtime.preview.types import ArmMount


class _StaticKinematics:
    home_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_observation_from_preview_arm() -> None:
    device = PreviewArmDevice(
        "elite_left",
        _StaticKinematics(),  # type: ignore[arg-type]
        ArmMount("elite_left", (0.0, 0.0, 0.0)),
        register=False,
    )
    device._q = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    obs = observation_from_preview_arm(device, device_id="elite_left")
    assert obs["online"] is True
    assert obs["idle"] is True
    assert len(obs["joint_positions_si"]) == 6
    assert math.isclose(math.degrees(obs["joint_positions_si"][0]), 10.0, abs_tol=1e-6)
