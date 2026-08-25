"""PLC、TCP/SDK 与 MoveIt 共用的机械臂维护运动合同。"""

from __future__ import annotations

import hashlib
import json
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Protocol, TypeAlias, runtime_checkable

from .commands import CommandResult
from .commissioning_pose import CommissioningPoseInput
from .observations import ObservationState
from .targets import CartesianPose

COMMISSIONING_PROTOCOL_VERSION = 2


class CommissioningMotionKind(str, Enum):
    """统一维护 Interface 允许的封闭运动命令类型。"""

    MOVE_TARGET = "move_target"
    MOVE_POSE = "move_pose"
    TCP_JOG = "tcp_jog"
    JOINT_JOG = "joint_jog"
    CONTROLLED_STOP = "controlled_stop"


class MotionDirection(str, Enum):
    """有限点动使用的正负方向，不允许调用方上传连续速度流。"""

    POSITIVE = "positive"
    NEGATIVE = "negative"

    @property
    def sign(self) -> float:
        """返回方向对应的数值符号。"""

        return 1.0 if self is MotionDirection.POSITIVE else -1.0


class TcpAxis(str, Enum):
    """工具中心点（TCP）允许点动的平移轴和旋转轴。"""

    X = "x"
    Y = "y"
    Z = "z"
    RX = "rx"
    RY = "ry"
    RZ = "rz"

    @property
    def rotational(self) -> bool:
        """旋转轴返回 ``True``，平移轴返回 ``False``。"""

        return self in {TcpAxis.RX, TcpAxis.RY, TcpAxis.RZ}


@dataclass(frozen=True, slots=True)
class CommissioningCapabilities:
    """一个 Adapter 声明的维护运动能力，不由 FE 或部署文件猜测。

    ``tcp_jog`` 为真意味着实现能读取当前 TCP 并做前后状态验证；
    ``joint_jog`` 为真意味着实现能读取完整关节状态、按 exact Arm 型号校验
    关节类型/限位，并验证非目标关节没有越过 profile 拥有的容差。
    """

    move_target: bool
    move_pose: bool
    tcp_jog: bool
    joint_jog: bool
    controlled_stop: bool

    def supports(self, kind: CommissioningMotionKind) -> bool:
        """返回指定维护命令是否由当前后端实现。"""

        return {
            CommissioningMotionKind.MOVE_TARGET: self.move_target,
            CommissioningMotionKind.MOVE_POSE: self.move_pose,
            CommissioningMotionKind.TCP_JOG: self.tcp_jog,
            CommissioningMotionKind.JOINT_JOG: self.joint_jog,
            CommissioningMotionKind.CONTROLLED_STOP: self.controlled_stop,
        }[kind]


@dataclass(frozen=True, slots=True)
class CommissioningJointPosition:
    """调试快照中的一个稳定关节位置。

    ``position_si`` 对转动关节为 rad，对移动关节为 m；关节类型和顺序由 exact
    Arm 型号拥有，不复制到点位资产或调试命令中。
    """

    joint_ref: str
    position_si: float

    def __post_init__(self) -> None:
        """校验稳定关节引用和有限 SI 数值。"""

        if not self.joint_ref.strip():
            raise ValueError("调试关节状态必须包含 joint_ref")
        if not math.isfinite(self.position_si):
            raise ValueError("调试关节状态 position_si 必须为有限数")


@dataclass(frozen=True, slots=True)
class CommissioningSnapshot:
    """统一维护 Interface 返回的当前机械臂只读快照。

    快照只提供调试准入和操作反馈，不是调度器（Scheduler）写模型，也不能证明
    物理结算（PhysicalSettlement）。PLC 可只返回控制器可观测的数据；只有在
    Adapter 声明相应 jog 能力时，所需关节/TCP 状态才是强制条件。
    """

    state: ObservationState
    observed_at: float
    max_age_s: float
    source: str
    online: bool | None
    idle: bool | None
    active_command_id: str | None
    execution_fenced: bool
    joint_positions: tuple[CommissioningJointPosition, ...] | None = None
    tcp_pose: CartesianPose | None = None

    def __post_init__(self) -> None:
        """校验时间、来源、活动命令和关节身份唯一性。"""

        if (
            not math.isfinite(self.observed_at)
            or not math.isfinite(self.max_age_s)
            or self.max_age_s <= 0.0
            or not self.source.strip()
        ):
            raise ValueError("调试快照必须包含有效时间、新鲜度与来源")
        if self.state is ObservationState.KNOWN and (
            self.online is None or self.idle is None
        ):
            raise ValueError("known 调试快照必须明确 online 与 idle")
        if self.active_command_id is not None and not self.active_command_id.strip():
            raise ValueError("active_command_id 不得为空字符串")
        if self.joint_positions is not None:
            refs = tuple(item.joint_ref for item in self.joint_positions)
            if len(refs) != len(set(refs)):
                raise ValueError("调试快照 joint_ref 不得重复")

    def is_fresh(self, now: float | None = None) -> bool:
        """按接收时钟判断该快照能否用于维护准入。"""

        checked_at = time.time() if now is None else now
        age = checked_at - self.observed_at
        return (
            self.state is ObservationState.KNOWN
            and 0.0 <= age <= self.max_age_s
        )


@dataclass(frozen=True, slots=True)
class _CommissioningCommandBase:
    """维护命令共享的幂等身份和活动硬件配置摘要。"""

    command_id: str
    hardware_profile_digest: str
    source_boot_id: str
    monotonic_sequence: int

    def __post_init__(self) -> None:
        """校验命令身份、配置摘要和单调序号。"""

        if any(
            not value.strip()
            for value in (
                self.command_id,
                self.hardware_profile_digest,
                self.source_boot_id,
            )
        ):
            raise ValueError("维护命令身份、HardwareProfile 与 boot_id 不能为空")
        if isinstance(self.monotonic_sequence, bool) or self.monotonic_sequence < 1:
            raise ValueError("维护命令 monotonic_sequence 必须是正整数")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回具体子类拥有的维护运动类型。"""

        raise NotImplementedError

    def canonical_payload(self) -> dict[str, object]:
        """返回包含命令类型的稳定 JSON 值，用于审计与幂等。"""

        payload: dict[str, object] = {
            "schema_version": COMMISSIONING_PROTOCOL_VERSION,
            "type": self.kind.value,
        }
        for name, value in asdict(self).items():
            payload[name] = _canonical_json_value(value)
        return payload

    def fingerprint(self) -> str:
        """返回绑定命令类型、目标和限制参数的 SHA-256 摘要。"""

        encoded = json.dumps(
            self.canonical_payload(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True, slots=True)
class _FiniteMotionCommand(_CommissioningCommandBase):
    """一步一命令的维护运动公共限制。"""

    motion_profile_ref: str
    velocity_scale: float
    acceleration_scale: float

    def __post_init__(self) -> None:
        """限制远程维护运动的速度和加速度缩放。"""

        _CommissioningCommandBase.__post_init__(self)
        if not self.motion_profile_ref.strip():
            raise ValueError("维护运动必须引用 motion_profile_ref")
        for name, value in (
            ("velocity_scale", self.velocity_scale),
            ("acceleration_scale", self.acceleration_scale),
        ):
            if not math.isfinite(value) or not 0.0 < value <= 0.30:
                raise ValueError(f"维护运动 {name} 必须位于 (0, 0.30]")


@dataclass(frozen=True, slots=True)
class MoveTargetCommand(_FiniteMotionCommand):
    """低速移动到一个已批准、版本化的稳定目标引用。"""

    target_ref: str
    target_revision: str

    def __post_init__(self) -> None:
        """校验目标引用和点位/程序资产版本。"""

        _FiniteMotionCommand.__post_init__(self)
        if not self.target_ref.strip() or not self.target_revision.strip():
            raise ValueError("move_target 必须包含 target_ref 与 target_revision")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回点位移动命令类型。"""

        return CommissioningMotionKind.MOVE_TARGET


@dataclass(frozen=True, slots=True)
class MovePoseCommand(_FiniteMotionCommand):
    """低速移动到操作员输入的一次临时绝对 TCP 位姿。

    该命令只用于维护运动，不保存、发布或替换版本化点位资产。未来 Adapter
    必须使用 ``pose_input.resolved_pose``，并用 ``tool_context_digest`` 证明
    规划与执行期间使用的是同一个快换工具/TCP 上下文。
    """

    pose_input: CommissioningPoseInput
    tool_context_digest: str

    def __post_init__(self) -> None:
        """校验结构化位姿输入和当前工具上下文摘要。"""

        _FiniteMotionCommand.__post_init__(self)
        if not isinstance(self.pose_input, CommissioningPoseInput):
            raise TypeError("move_pose 必须包含 CommissioningPoseInput")
        if not self.tool_context_digest.strip():
            raise ValueError("move_pose 必须绑定 tool_context_digest")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回临时绝对位姿移动命令类型。"""

        return CommissioningMotionKind.MOVE_POSE


@dataclass(frozen=True, slots=True)
class TcpJogCommand(_FiniteMotionCommand):
    """在基座或工具坐标系中执行一次 TCP 点动。"""

    frame_ref: str
    axis: TcpAxis
    direction: MotionDirection
    step_si: float

    def __post_init__(self) -> None:
        """校验坐标系以及正的有限 SI 步长。"""

        _FiniteMotionCommand.__post_init__(self)
        if self.frame_ref not in {"arm_base", "tool"}:
            raise ValueError("tcp_jog frame_ref 只允许 arm_base 或 tool")
        if not math.isfinite(self.step_si) or self.step_si <= 0.0:
            raise ValueError("tcp_jog step_si 必须为正的有限数")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回 TCP 点动命令类型。"""

        return CommissioningMotionKind.TCP_JOG


@dataclass(frozen=True, slots=True)
class JointJogCommand(_FiniteMotionCommand):
    """只改变 exact Arm 型号中一个稳定关节引用的有限点动。

    ``step_si`` 的量纲由型号中的关节类型决定：转动关节为 rad，移动关节为 m。
    协议不重复 ``joint_names``、关节类型或单位；关节限位以及其余关节锁定容差
    必须由 ``motion_profile_ref`` 与 exact Arm 型号共同校验。
    """

    joint_ref: str
    direction: MotionDirection
    step_si: float

    def __post_init__(self) -> None:
        """校验稳定关节引用和正的有限 SI 步长。"""

        _FiniteMotionCommand.__post_init__(self)
        if not self.joint_ref.strip():
            raise ValueError("joint_jog 必须包含 joint_ref")
        if not math.isfinite(self.step_si) or self.step_si <= 0.0:
            raise ValueError("joint_jog step_si 必须为正的有限数")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回单关节点动命令类型。"""

        return CommissioningMotionKind.JOINT_JOG


@dataclass(frozen=True, slots=True)
class ControlledStopCommand(_CommissioningCommandBase):
    """请求停止一个已有维护运动，不把普通停止冒充急停。"""

    target_command_id: str
    reason: str

    def __post_init__(self) -> None:
        """校验待停止命令身份和审计原因。"""

        _CommissioningCommandBase.__post_init__(self)
        if not self.target_command_id.strip() or not self.reason.strip():
            raise ValueError("controlled_stop 必须包含目标命令和原因")

    @property
    def kind(self) -> CommissioningMotionKind:
        """返回受控停止命令类型。"""

        return CommissioningMotionKind.CONTROLLED_STOP


CommissioningCommand: TypeAlias = (
    MoveTargetCommand
    | MovePoseCommand
    | TcpJogCommand
    | JointJogCommand
    | ControlledStopCommand
)


def _canonical_json_value(value: object) -> object:
    """把嵌套数据类值转换为稳定、可 JSON 序列化的协议值。

    参数：``value`` 是 ``asdict`` 产生的任意嵌套值。
    返回：枚举已替换为 wire value、序列和映射已递归规范化的值。
    """

    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {
            str(key): _canonical_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [_canonical_json_value(item) for item in value]
    return value


@runtime_checkable
class RobotCommissioningPort(Protocol):
    """PLC、TCP/SDK 与 MoveIt 共用的唯一维护运动 Interface。"""

    @property
    def commissioning_capabilities(self) -> CommissioningCapabilities:
        """返回当前 Adapter 经实现验证的能力集合。"""

    @property
    def commissioning_target_revision(self) -> str | None:
        """返回 move_target 当前活动的不可变 PointSet/程序集版本。"""

    def commissioning_snapshot(self) -> CommissioningSnapshot:
        """读取只读当前状态；未知、过期或 Fence 必须由调用方失败关闭。"""

    def execute_commissioning(
        self,
        command: CommissioningCommand,
    ) -> CommandResult:
        """执行一个封闭维护命令并返回精确命令结果。

        实现必须限制在 maintenance/simulation profile、端点独占和硬件许可内；
        派发结果不明时返回 ``execution_unknown`` 并保留 Fence，禁止自动重放。
        """


__all__ = [
    "COMMISSIONING_PROTOCOL_VERSION",
    "CommissioningCapabilities",
    "CommissioningCommand",
    "CommissioningJointPosition",
    "CommissioningMotionKind",
    "CommissioningSnapshot",
    "ControlledStopCommand",
    "JointJogCommand",
    "MotionDirection",
    "MovePoseCommand",
    "MoveTargetCommand",
    "RobotCommissioningPort",
    "TcpAxis",
    "TcpJogCommand",
]
