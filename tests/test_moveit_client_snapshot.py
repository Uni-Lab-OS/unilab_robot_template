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


class _FailedTerminalClient(_SlowFkClient):
    """wait_until_executed 返回明确失败，不是通信歧义。"""

    def compute_fk(self, **kwargs: Any) -> SimpleNamespace:
        del kwargs
        return SimpleNamespace(pose=_Pose())

    def move_to_configuration(self, **kwargs: Any) -> None:
        del kwargs

    def move_to_pose(self, **kwargs: Any) -> None:
        del kwargs

    def wait_until_executed(self) -> bool:
        return False


def _assert_known_failed_receipt(port: Any) -> None:
    receipt = port.execute_joint_target(
        group_name="arm",
        joint_names=port.qualified_joint_names,
        target=(0.0,) * 6,
        command_id="cmd-failed",
        parameters={},
    )
    assert receipt["state"] == "failed"
    assert receipt["completed"] is False
    assert receipt["command_id"] == "cmd-failed"


def test_cr5_known_moveit_failure_returns_failed_receipt() -> None:
    """MoveIt 已知失败终态必须返回 failed，不能抛成派发不明。"""

    names = tuple(f"robot_cr5_joint_{index}" for index in range(1, 7))
    port = Cr5MoveIt2ClientPort(
        _FailedTerminalClient(names), qualified_joint_names=names
    )
    _assert_known_failed_receipt(port)


def test_cr7_known_moveit_failure_returns_failed_receipt() -> None:
    """CR7 与 CR5 共用同一已知失败终态合同。"""

    names = tuple(f"robot_cr7_joint_{index}" for index in range(1, 7))
    port = Cr7MoveIt2ClientPort(
        _FailedTerminalClient(names), qualified_joint_names=names
    )
    _assert_known_failed_receipt(port)
