"""Preview TCP 点动。"""

from __future__ import annotations

import math

from unilab_robot_runtime.preview.preview_arm_device import PreviewArmDevice
from unilab_robot_runtime.preview.types import ArmMount
from unilab_arm_elite_cs66 import EliteCs66PreviewKinematics


def test_preview_tcp_jog_translates_tcp_position() -> None:
    kinematics = EliteCs66PreviewKinematics()
    mount = ArmMount("elite_left", (0.0, 0.0, 0.0))
    device = PreviewArmDevice("elite_left", kinematics, mount, register=False)
    device.moveJ(*kinematics.home_deg, duration=0.1)
    before = kinematics.fk_tcp(mount, device.joints_deg)[:3, 3].copy()
    result = device.jog_tcp_once("x", "positive", step=10.0, frame_ref="arm_base", duration=0.1)
    after = kinematics.fk_tcp(mount, device.joints_deg)[:3, 3]
    assert result["success"] is True
    assert math.isclose(float(after[0] - before[0]), 0.01, abs_tol=0.002)
