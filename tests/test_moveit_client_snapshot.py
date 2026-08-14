"""MoveIt2 调试快照必须按接收时钟判断新鲜度。"""

from __future__ import annotations

import time
from types import SimpleNamespace
from typing import Any

from unilab_arm_cr5.adapters import MoveIt2ClientPort as Cr5MoveIt2ClientPort
from unilab_arm_cr7.adapters import MoveIt2ClientPort as Cr7MoveIt2ClientPort


class _Pose:
    position = SimpleNamespace(x=0.4, y=0.0, z=0.3)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _SlowFkClient:
    """关节 stamp 故意不是墙钟，且 FK 慢于 max_age_s。"""

    def __init__(self, names: tuple[str, ...]) -> None:
        self.joint_state = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            name=names,
            position=(0.0,) * len(names),
        )

    def compute_fk(self, **kwargs: Any) -> SimpleNamespace:
        del kwargs
        time.sleep(0.6)
        return SimpleNamespace(pose=_Pose())

    def query_state(self) -> SimpleNamespace:
        return SimpleNamespace(name="IDLE")


def _assert_receive_clock_snapshot(port: Any) -> None:
    before = time.time()
    raw = port.read_commissioning_state()
    after = time.time()

    assert before <= float(raw["observed_at"]) <= after
    assert float(raw["observed_at"]) > 1_000_000_000.0
    assert after - float(raw["observed_at"]) <= 0.5
    assert raw["online"] is True
    assert raw["idle"] is True


def test_cr5_commissioning_snapshot_uses_receive_clock_not_ros_stamp() -> None:
    """ROS header stamp 过期或 FK 变慢时，调试快照仍应按接收时刻准入。"""

    names = tuple(f"robot_cr5_joint_{index}" for index in range(1, 7))
    port = Cr5MoveIt2ClientPort(_SlowFkClient(names), qualified_joint_names=names)
    _assert_receive_clock_snapshot(port)


def test_cr7_commissioning_snapshot_uses_receive_clock_not_ros_stamp() -> None:
    """CR7 与 CR5 共用同一接收时钟合同，禁止把 header stamp 当 observed_at。"""

    names = tuple(f"robot_cr7_joint_{index}" for index in range(1, 7))
    port = Cr7MoveIt2ClientPort(_SlowFkClient(names), qualified_joint_names=names)
    _assert_receive_clock_snapshot(port)
