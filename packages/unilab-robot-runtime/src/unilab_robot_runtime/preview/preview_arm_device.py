"""Host 进程本地 preview 机械臂；无硬件 I/O。"""

from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING

from .joint_interpolator import interpolate_joint_path, smoothstep_ratio, validate_duration
from .preview_registry import LOCAL_ARMS
from .types import ArmMount, MotionResult, StopResult

if TYPE_CHECKING:
    from .protocols import PreviewKinematics


class PreviewArmDevice:
    """通用 preview 机械臂设备核心；不含 @device 装饰。"""

    mode = "local_preview"

    def __init__(
        self,
        device_id: str,
        kinematics: PreviewKinematics,
        mount: ArmMount,
        *,
        register: bool = True,
    ) -> None:
        self.device_id = device_id
        self._kinematics = kinematics
        self._mount = mount
        self._q = list(kinematics.home_deg)
        self._lock = threading.RLock()
        self._motion = threading.Lock()
        self._stop = threading.Event()
        self._publisher = None
        self._ros = None
        self.grip = 0.0
        self.rail = 0.0
        if register and device_id:
            LOCAL_ARMS[device_id] = self

    def post_init(self, ros_node=None) -> None:
        if ros_node is not None:
            from sensor_msgs.msg import JointState

            self._message_type = JointState
            self._ros = ros_node
            self._publisher = ros_node.create_publisher(JointState, "/joint_states", 10)
            ros_node.create_timer(0.05, self._publish)
            self._publish()

    def _publish(self) -> None:
        if self._publisher is None:
            return
        msg = self._message_type()
        msg.header.stamp = self._ros.get_clock().now().to_msg()
        msg.name = [f"{self.device_id}_joint_{i}" for i in range(1, 7)] + [
            self.device_id + "_jaw_left",
            self.device_id + "_jaw_right",
            self.device_id + "_rail_y",
        ]
        with self._lock:
            msg.position = [math.radians(v) for v in self._q] + [
                self.grip,
                self.grip,
                self.rail,
            ]
        self._publisher.publish(msg)

    def move_rail(self, target: float, duration: float, external_stop: threading.Event) -> None:
        limits = self._mount.rail_limits
        if limits is None:
            raise ValueError(f"{self.device_id} 未配置导轨限位")
        lo, hi = limits
        base_y = self._mount.base_xyz[1]
        if not lo - 1e-8 <= base_y + target <= hi + 1e-8:
            raise ValueError("Rail candidate range exceeded")
        start = self.rail
        count = max(2, math.ceil(duration / 0.05))
        for i in range(1, count + 1):
            if external_stop.wait(duration / count):
                raise RuntimeError("Rail motion stopped")
            self.rail = float(start + (target - start) * i / count)
            self._publish()

    @property
    def joints_deg(self) -> list[float]:
        with self._lock:
            return list(self._q)

    def follow_path(
        self,
        path,
        duration: float,
        external_stop: threading.Event,
    ) -> None:
        def on_step(q: list[float]) -> None:
            with self._lock:
                self._q = q
            self._publish()

        interpolate_joint_path(
            path,
            duration,
            stop_event=self._stop,
            external_stop=external_stop,
            on_step=on_step,
        )

    def moveJ(
        self,
        j1: float = 90.0,
        j2: float = -65.0,
        j3: float = -70.0,
        j4: float = -135.0,
        j5: float = 90.0,
        j6: float = 0.0,
        duration: float = 2.0,
    ) -> MotionResult:
        target = [float(v) for v in (j1, j2, j3, j4, j5, j6)]
        if any(not math.isfinite(v) or abs(v) > 360 for v in target):
            raise ValueError("each joint must be within +/-360 deg")
        validate_duration(duration)
        if not self._motion.acquire(blocking=False):
            raise RuntimeError("arm is busy")
        try:
            self._stop.clear()
            start = self.joints_deg
            steps = max(1, math.ceil(duration / 0.05))
            for i in range(1, steps + 1):
                if self._stop.wait(duration / steps):
                    return {
                        "status": "stopped",
                        "joints_deg": self.joints_deg,
                        "mode": self.mode,
                    }
                ratio = smoothstep_ratio(i, steps)
                with self._lock:
                    self._q = [a + (b - a) * ratio for a, b in zip(start, target)]
                self._publish()
            return {
                "status": "completed",
                "joints_deg": self.joints_deg,
                "mode": self.mode,
            }
        finally:
            self._motion.release()

    def set_joint(self, joint: int = 1, angle: float = 90.0, duration: float = 1.0) -> MotionResult:
        if type(joint) is not int or not 1 <= joint <= 6:
            raise ValueError("joint must be an integer from 1 to 6")
        q = self.joints_deg
        q[joint - 1] = angle
        return self.moveJ(*q, duration=duration)

    def jog_tcp_once(
        self,
        axis: str,
        direction: str,
        step: float = 1.0,
        frame_ref: str = "arm_base",
        duration: float = 1.0,
    ) -> dict[str, object]:
        from .tcp_jog import jog_tcp_once_preview

        return jog_tcp_once_preview(
            self,
            axis=axis,
            direction=direction,
            step=step,
            frame_ref=frame_ref,
            duration=duration,
        )

    def home(self, duration: float = 2.0) -> MotionResult:
        return self.moveJ(*self._kinematics.home_deg, duration=duration)

    def stop(self) -> StopResult:
        self._stop.set()
        return {"status": "stop_requested", "mode": self.mode}
